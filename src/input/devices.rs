use std::any::TypeId;
use std::collections::HashMap;
use std::ffi::{CStr, CString};
use std::sync::Mutex;
#[cfg(feature = "monado")]
use std::time::{Duration, Instant};

use glam::Mat4;
use openvr as vr;
use openxr as xr;
use openxr::sys::Handle;

use crate::input::profiles::knuckles::Knuckles;
#[cfg(feature = "monado")]
use crate::input::profiles::vive_tracker::ViveTracker;
#[cfg(feature = "monado")]
use openxr_mndx_xdev_space::{SessionXDevExtensionMNDX, XDev, XR_MNDX_XDEV_SPACE_EXTENSION_NAME};

use crate::input::profiles::ProfileProperties;
use crate::openxr_data::{self, Hand, OpenXrData, SessionData};
use crate::tracy_span;
use log::trace;

use super::{Input, InteractionProfile, profiles::MainAxisType};

pub enum TrackedDeviceType {
    Hmd,
    Controller {
        hand: Hand,
        hand_tracker: Option<xr::HandTracker>,
        skeleton_cache: Mutex<HashMap<u64, Option<xr::HandJointLocations>>>,
    },
    #[cfg(feature = "monado")]
    GenericTracker {
        space: Option<xr::Space>,
        serial: CString,
    },
}

impl std::fmt::Debug for TrackedDeviceType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TrackedDeviceType::Hmd => write!(f, "HMD"),
            TrackedDeviceType::Controller { hand, .. } => write!(f, "Controller ({:?})", hand),
            #[cfg(feature = "monado")]
            TrackedDeviceType::GenericTracker { serial, .. } => {
                write!(f, "Generic Tracker ({})", serial.to_string_lossy())
            }
        }
    }
}

#[derive(Clone, Copy)]
pub struct ProfileData {
    properties: &'static ProfileProperties,
    get_hand_offset: fn(Hand) -> Mat4,
    /// For Knuckles, the skeleton thumb tries to accurately match where the physical
    /// thumb is, e.g. the curl depends on which part of the touchpad is being touched,
    /// or how the thumbstick is being pushed, but in GetSkeletalSummaryData with
    /// EVRSummaryType::FromDevice the curl is set to 1 if any of touchpad, A/B,
    /// or thumbstick is being touched, so just use the skeletal input actions to
    /// determine the curl value for the thumb.
    pub force_estimated_thumb: bool,
}

impl ProfileData {
    pub fn new<P: InteractionProfile>() -> Self {
        Self {
            properties: P::properties(),
            get_hand_offset: P::offset_grip_pose,
            force_estimated_thumb: TypeId::of::<P>() == TypeId::of::<Knuckles>(),
        }
    }

    #[inline]
    pub fn hand_offset(&self, hand: Hand) -> Mat4 {
        (self.get_hand_offset)(hand)
    }
}

pub struct TrackedDevice {
    device_type: TrackedDeviceType,
    pub profile_data: Option<ProfileData>,
    pub profile_path: xr::Path,
    pub connected: bool,
    pub previous_connected: bool,
    pose_cache: Mutex<Option<vr::TrackedDevicePose_t>>,
}

fn get_hmd_pose(
    xr_data: &OpenXrData<impl crate::openxr_data::Compositor>,
    session_data: &SessionData,
    origin: vr::ETrackingUniverseOrigin,
) -> Option<vr::TrackedDevicePose_t> {
    let (location, velocity) = {
        session_data
            .view_space
            .relate(
                session_data.get_space_for_origin(origin),
                xr_data.display_time.get(),
            )
            .ok()?
    };

    Some(vr::space_relation_to_openvr_pose(location, velocity))
}

fn get_controller_pose(
    xr_data: &OpenXrData<impl crate::openxr_data::Compositor>,
    session_data: &SessionData,
    controller: &TrackedDevice,
    origin: vr::ETrackingUniverseOrigin,
) -> Option<vr::TrackedDevicePose_t> {
    let pose_data = session_data.input_data.pose_data.get()?;

    let spaces = match controller.get_controller_hand().unwrap() {
        Hand::Left => &pose_data.left_space,
        Hand::Right => &pose_data.right_space,
    };

    let (location, velocity) = if let Some(raw) =
        spaces.try_get_or_init_raw(&controller.profile_data, session_data, pose_data)
    {
        raw.relate(
            session_data.get_space_for_origin(origin),
            xr_data.display_time.get(),
        )
        .ok()?
    } else {
        trace!("Failed to get raw space, returning empty pose");
        (xr::SpaceLocation::default(), xr::SpaceVelocity::default())
    };

    Some(vr::space_relation_to_openvr_pose(location, velocity))
}

#[cfg(feature = "monado")]
fn get_generic_tracker_pose(
    xr_data: &OpenXrData<impl crate::openxr_data::Compositor>,
    session_data: &SessionData,
    tracker: &TrackedDevice,
    origin: vr::ETrackingUniverseOrigin,
) -> Option<vr::TrackedDevicePose_t> {
    let TrackedDeviceType::GenericTracker { space, .. } = tracker.get_type() else {
        return None;
    };

    let (location, velocity) = space
        .as_ref()?
        .relate(
            session_data.get_space_for_origin(origin),
            xr_data.display_time.get(),
        )
        .ok()?;

    let mut pose = vr::space_relation_to_openvr_pose(location, velocity);
    if !location.location_flags.contains(
        xr::SpaceLocationFlags::POSITION_TRACKED | xr::SpaceLocationFlags::ORIENTATION_TRACKED,
    ) {
        pose.eTrackingResult = vr::ETrackingResult::Running_OutOfRange;
    }
    Some(pose)
}

impl TrackedDevice {
    pub(super) fn new(
        device_type: TrackedDeviceType,
        profile_path: Option<xr::Path>,
        profile_data: Option<ProfileData>,
    ) -> Self {
        Self {
            profile_data,
            profile_path: profile_path.unwrap_or(xr::Path::NULL),
            connected: matches!(device_type, TrackedDeviceType::Hmd),
            device_type,
            previous_connected: false,
            pose_cache: Mutex::new(None),
        }
    }

    pub fn get_pose(
        &self,
        xr_data: &OpenXrData<impl crate::openxr_data::Compositor>,
        session_data: &SessionData,
        origin: vr::ETrackingUniverseOrigin,
    ) -> Option<vr::TrackedDevicePose_t> {
        let mut pose_cache = self.pose_cache.lock().unwrap();
        if let Some(pose) = *pose_cache {
            return Some(pose);
        }

        *pose_cache = match self.device_type {
            TrackedDeviceType::Hmd => get_hmd_pose(xr_data, session_data, origin),
            TrackedDeviceType::Controller { .. } => {
                get_controller_pose(xr_data, session_data, self, origin)
            }
            #[cfg(feature = "monado")]
            TrackedDeviceType::GenericTracker { .. } => {
                get_generic_tracker_pose(xr_data, session_data, self, origin)
            }
        };

        *pose_cache
    }

    #[cfg(feature = "monado")]
    pub(super) fn update_tracker_connection(
        &mut self,
        xr_data: &OpenXrData<impl crate::openxr_data::Compositor>,
        session_data: &SessionData,
    ) {
        if !matches!(self.device_type, TrackedDeviceType::GenericTracker { .. }) {
            return;
        }
        // OpenXR exposes pose validity, not a tracker's physical power state.
        // Use LOCAL so stage calibration cannot make a connected tracker disappear.
        // Valid but untracked (held/inferred) poses remain connected. If the runtime
        // also invalidates the pose on power-off, mirror that in OpenVR's device
        // state and activation/deactivation events, keeping the slot for its return.
        self.connected = get_generic_tracker_pose(
            xr_data,
            session_data,
            self,
            vr::ETrackingUniverseOrigin::Seated,
        )
        .is_some_and(|pose| pose.bDeviceIsConnected);
    }

    pub fn get_hand_skeleton(
        &self,
        xr_data: &OpenXrData<impl crate::openxr_data::Compositor>,
        base: &xr::Space,
    ) -> Option<xr::HandJointLocations> {
        let TrackedDeviceType::Controller {
            hand_tracker,
            skeleton_cache,
            ..
        } = self.get_type()
        else {
            return None;
        };
        let mut skeleton_cache = skeleton_cache.lock().unwrap();
        if let Some(skeleton) = skeleton_cache.get(&base.as_raw().into_raw()) {
            return *skeleton;
        }

        let joints = base
            .locate_hand_joints(hand_tracker.as_ref()?, xr_data.display_time.get())
            .unwrap_or_default();
        skeleton_cache.insert(base.as_raw().into_raw(), joints);
        joints
    }

    pub fn clear_pose_cache(&self) {
        std::mem::take(&mut *self.pose_cache.lock().unwrap());
        if let TrackedDeviceType::Controller { skeleton_cache, .. } = self.get_type() {
            skeleton_cache.lock().unwrap().clear();
        }
    }

    pub fn has_connected_changed(&mut self) -> bool {
        if self.previous_connected != self.connected {
            self.previous_connected = self.connected;
            true
        } else {
            false
        }
    }

    pub fn get_type(&self) -> &TrackedDeviceType {
        &self.device_type
    }

    pub fn get_controller_hand(&self) -> Option<Hand> {
        match self.device_type {
            TrackedDeviceType::Controller { hand, .. } => Some(hand),
            _ => None,
        }
    }

    fn get_string_property(&self, property: vr::ETrackedDeviceProperty) -> Option<&CStr> {
        let hand = match self.device_type {
            TrackedDeviceType::Controller { hand, .. } => hand,
            _ => Hand::Left,
        };

        let data = self.profile_data.as_ref()?.properties;

        match property {
            // Audica likes to apply controller specific tweaks via this property
            vr::ETrackedDeviceProperty::ControllerType_String => Some(data.openvr_controller_type),
            // I Expect You To Die 3 identifies controllers with this property -
            // why it couldn't just use ControllerType instead is beyond me...
            // Because some controllers have different model names for each hand......
            vr::ETrackedDeviceProperty::ModelNumber_String => Some(*data.model.get(hand)),
            // Resonite won't recognize controllers without this
            vr::ETrackedDeviceProperty::RenderModelName_String => {
                Some(*data.render_model_name.get(hand))
            }
            vr::ETrackedDeviceProperty::RegisteredDeviceType_String => {
                Some(*data.registered_device_type.get(hand))
            }
            vr::ETrackedDeviceProperty::TrackingSystemName_String => {
                Some(data.tracking_system_name)
            }
            // Required for controllers to be acknowledged in I Expect You To Die 3
            vr::ETrackedDeviceProperty::SerialNumber_String => match self.get_type() {
                TrackedDeviceType::Controller { .. } => Some(*data.serial_number.get(hand)),
                #[cfg(feature = "monado")]
                TrackedDeviceType::GenericTracker { serial, .. } => Some(serial.as_c_str()),
                TrackedDeviceType::Hmd => unreachable!(),
            },
            vr::ETrackedDeviceProperty::ManufacturerName_String => Some(data.manufacturer_name),
            _ => None,
        }
    }

    fn get_int_property(&self, property: vr::ETrackedDeviceProperty) -> Option<i32> {
        match self.device_type {
            TrackedDeviceType::Controller { .. } => {
                let data = self.profile_data.as_ref()?.properties;

                match property {
                    vr::ETrackedDeviceProperty::Axis0Type_Int32 => match data.main_axis {
                        MainAxisType::Thumbstick => Some(vr::EVRControllerAxisType::Joystick as _),
                        MainAxisType::Trackpad => Some(vr::EVRControllerAxisType::TrackPad as _),
                    },
                    vr::ETrackedDeviceProperty::Axis1Type_Int32 => {
                        Some(vr::EVRControllerAxisType::Trigger as _)
                    }
                    vr::ETrackedDeviceProperty::Axis2Type_Int32 => {
                        // This is actually the grip, and gets recognized as such
                        Some(vr::EVRControllerAxisType::Trigger as _)
                    }
                    // TODO: report knuckles trackpad?
                    vr::ETrackedDeviceProperty::Axis3Type_Int32
                    | vr::ETrackedDeviceProperty::Axis4Type_Int32 => {
                        Some(vr::EVRControllerAxisType::None as _)
                    }
                    _ => None,
                }
            }
            _ => None,
        }
    }

    fn get_uint_property(&self, property: vr::ETrackedDeviceProperty) -> Option<u64> {
        match self.device_type {
            TrackedDeviceType::Controller { .. } => {
                let data = self.profile_data.as_ref()?.properties;

                match property {
                    vr::ETrackedDeviceProperty::SupportedButtons_Uint64 => {
                        Some(data.legacy_buttons_mask)
                    }
                    _ => None,
                }
            }
            _ => None,
        }
    }
}

pub struct SubactionPaths {
    pub left: xr::Path,
    pub right: xr::Path,
}

impl SubactionPaths {
    pub fn new(instance: &xr::Instance) -> Self {
        let left = instance
            .string_to_path("/user/hand/left")
            .expect("Failed to convert string to path");
        let right = instance
            .string_to_path("/user/hand/right")
            .expect("Failed to convert string to path");

        Self { left, right }
    }
}

pub struct TrackedDeviceList {
    devices: Vec<TrackedDevice>,
    #[cfg(feature = "monado")]
    last_tracker_refresh: Option<Instant>,
}

/// A device list's slots and states without what belonged to the session they
/// were made in (hand trackers, tracker spaces): what a session restart carries
/// over, so devices keep their indices and the game sees no disconnect.
pub struct DeviceLayout(Vec<DeviceSlot>);

struct DeviceSlot {
    device_type: DeviceSlotType,
    profile_data: Option<ProfileData>,
    profile_path: xr::Path,
    connected: bool,
    previous_connected: bool,
}

enum DeviceSlotType {
    Hmd,
    Controller(Hand),
    #[cfg(feature = "monado")]
    GenericTracker(CString),
}

/// A hand tracker for `hand`, where the runtime has hand tracking.
pub(super) fn create_hand_tracker(
    session: &xr::Session<xr::AnyGraphics>,
    hand: Hand,
) -> Option<xr::HandTracker> {
    session
        .create_hand_tracker(hand.into())
        .inspect_err(|e| {
            if !matches!(
                *e,
                xr::sys::Result::ERROR_EXTENSION_NOT_PRESENT
                    | xr::sys::Result::ERROR_FEATURE_UNSUPPORTED
            ) {
                log::warn!("Failed to create hand tracker for hand {hand:?}: {e}");
            }
        })
        .ok()
}

impl Default for TrackedDeviceList {
    fn default() -> Self {
        Self {
            devices: vec![TrackedDevice::new(TrackedDeviceType::Hmd, None, None)],
            #[cfg(feature = "monado")]
            last_tracker_refresh: None,
        }
    }
}

impl TrackedDeviceList {
    /// Preserve every slot, including trackers discovered before controllers.
    pub(super) fn layout(&self) -> DeviceLayout {
        DeviceLayout(
            self.devices
                .iter()
                .map(|d| {
                    let device_type = match &d.device_type {
                        TrackedDeviceType::Hmd => DeviceSlotType::Hmd,
                        TrackedDeviceType::Controller { hand, .. } => {
                            DeviceSlotType::Controller(*hand)
                        }
                        #[cfg(feature = "monado")]
                        TrackedDeviceType::GenericTracker { serial, .. } => {
                            DeviceSlotType::GenericTracker(serial.clone())
                        }
                    };
                    DeviceSlot {
                        device_type,
                        profile_data: d.profile_data,
                        profile_path: d.profile_path,
                        connected: d.connected,
                        previous_connected: d.previous_connected,
                    }
                })
                .collect(),
        )
    }

    /// The devices of `layout`, made again for a new session.
    pub(super) fn from_layout(
        layout: DeviceLayout,
        session: &xr::Session<xr::AnyGraphics>,
    ) -> Self {
        if layout.0.is_empty() {
            return Self::default();
        }
        let devices = layout
            .0
            .into_iter()
            .map(|slot| {
                let device_type = match slot.device_type {
                    DeviceSlotType::Hmd => TrackedDeviceType::Hmd,
                    DeviceSlotType::Controller(hand) => TrackedDeviceType::Controller {
                        hand,
                        hand_tracker: create_hand_tracker(session, hand),
                        skeleton_cache: Mutex::new(Default::default()),
                    },
                    #[cfg(feature = "monado")]
                    DeviceSlotType::GenericTracker(serial) => TrackedDeviceType::GenericTracker {
                        serial,
                        space: None,
                    },
                };
                let mut device =
                    TrackedDevice::new(device_type, Some(slot.profile_path), slot.profile_data);
                device.connected = slot.connected;
                device.previous_connected = slot.previous_connected;
                device
            })
            .collect();
        Self {
            devices,
            #[cfg(feature = "monado")]
            last_tracker_refresh: None,
        }
    }

    pub(super) fn get_device(
        &self,
        device_index: vr::TrackedDeviceIndex_t,
    ) -> Option<&TrackedDevice> {
        self.devices.get(device_index as usize)
    }
    pub(super) fn get_device_mut(
        &mut self,
        device_index: vr::TrackedDeviceIndex_t,
    ) -> Option<&mut TrackedDevice> {
        self.devices.get_mut(device_index as usize)
    }

    pub(super) fn push_device(
        &mut self,
        device: TrackedDevice,
    ) -> Result<vr::TrackedDeviceIndex_t, vr::EVRInputError> {
        let index = self.devices.len() as vr::TrackedDeviceIndex_t;

        if index >= vr::k_unMaxTrackedDeviceCount {
            return Err(vr::EVRInputError::MaxCapacityReached);
        }

        self.devices.push(device);

        Ok(index)
    }

    pub(super) fn get_controller(&self, hand: Hand) -> Option<&TrackedDevice> {
        self.get_device(self.get_controller_index(hand)?)
    }
    pub(super) fn get_controller_mut(&mut self, hand: Hand) -> Option<&mut TrackedDevice> {
        self.get_device_mut(self.get_controller_index(hand)?)
    }

    fn get_controller_index(&self, hand: Hand) -> Option<vr::TrackedDeviceIndex_t> {
        self.iter()
            .enumerate()
            .find(|(_, device)| device.get_controller_hand() == Some(hand))
            .map(|(i, _)| i as vr::TrackedDeviceIndex_t)
    }

    #[cfg(feature = "monado")]
    pub(super) fn create_monado_generic_trackers(
        &mut self,
        xr_data: &OpenXrData<impl crate::openxr_data::Compositor>,
        session_data: &SessionData,
    ) -> xr::Result<()> {
        self.last_tracker_refresh = Some(Instant::now());
        if !xr_data
            .enabled_extensions
            .other
            .contains(&[XR_MNDX_XDEV_SPACE_EXTENSION_NAME.as_bytes(), b"\0"].concat())
        {
            return Ok(());
        }

        let extra_tracker_serials = std::env::var("XRIZER_TRACKER_SERIALS")
            .map_or(vec![], |trackers| {
                trackers.split(";").map(|t| t.to_string()).collect()
            });

        let xdevs: Vec<XDev> = session_data
            .session
            .get_xdev_list()?
            .enumerate_xdevs()?
            .into_iter()
            .filter(|xdev| {
                xdev.can_create_space()
                    && (xdev.name().to_lowercase().contains("tracker")
                        || extra_tracker_serials.contains(&xdev.serial().to_string()))
            })
            .collect();

        // Keep slots stable when the runtime stops listing a device. A returning
        // serial reuses its old slot, even if other devices have since arrived.
        for device in &mut self.devices {
            let TrackedDeviceType::GenericTracker { serial, space } = &mut device.device_type
            else {
                continue;
            };
            if !xdevs
                .iter()
                .any(|xdev| xdev.serial().as_bytes() == serial.as_bytes())
            {
                *space = None;
                device.connected = false;
            }
        }

        for xdev in xdevs {
            let existing = self.devices.iter().position(|device| {
                matches!(&device.device_type, TrackedDeviceType::GenericTracker { serial, .. }
                    if xdev.serial().as_bytes() == serial.as_bytes())
            });
            if let Some(index) = existing {
                let device = &mut self.devices[index];
                let TrackedDeviceType::GenericTracker { space, .. } = &mut device.device_type
                else {
                    unreachable!();
                };
                if space.is_none() {
                    match xdev.create_space(xr::Posef::IDENTITY) {
                        Ok(new_space) => {
                            *space = Some(new_space);
                        }
                        Err(e) => log::warn!("Couldn't recreate tracker {}: {e}", xdev.serial()),
                    }
                }
                continue;
            }
            if self.devices.len() >= vr::k_unMaxTrackedDeviceCount as usize {
                continue;
            }
            let space = match xdev.create_space(xr::Posef::IDENTITY) {
                Ok(space) => space,
                Err(e) => {
                    log::warn!("Couldn't create tracker {}: {e}", xdev.serial());
                    continue;
                }
            };
            let serial = CString::new(xdev.serial()).unwrap();
            log::info!(
                "Adding generic tracker {} at device index {} via Monado XDev extension",
                xdev.serial(),
                self.devices.len()
            );
            let tracker = TrackedDevice::new(
                TrackedDeviceType::GenericTracker {
                    serial,
                    space: Some(space),
                },
                None,
                Some(ProfileData::new::<ViveTracker>()),
            );
            self.devices.push(tracker);
        }

        Ok(())
    }

    #[cfg(feature = "monado")]
    pub(super) fn refresh_monado_generic_trackers(
        &mut self,
        xr_data: &OpenXrData<impl crate::openxr_data::Compositor>,
        session_data: &SessionData,
    ) {
        // Tracker hotplug need not change either hand's interaction profile.
        // XDev lists are snapshots, so periodically request a fresh one.
        if self
            .last_tracker_refresh
            .is_none_or(|last| last.elapsed() >= Duration::from_secs(1))
        {
            self.create_monado_generic_trackers(xr_data, session_data)
                .unwrap_or_else(|e| log::warn!("Couldn't refresh generic trackers: {e}"));
        }
    }

    pub fn iter(&self) -> impl Iterator<Item = &TrackedDevice> {
        self.devices.iter()
    }
    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut TrackedDevice> {
        self.devices.iter_mut()
    }
}

impl<C: openxr_data::Compositor> Input<C> {
    pub fn get_poses(
        &self,
        poses: &mut [vr::TrackedDevicePose_t],
        origin: Option<vr::ETrackingUniverseOrigin>,
    ) {
        tracy_span!();
        let session_data = self.openxr.session_data.get();
        let devices = session_data.input_data.devices.read().unwrap();

        for (i, pose) in poses.iter_mut().enumerate() {
            let device = devices.get_device(i as u32);

            if let Some(device) = device {
                *pose = device
                    .get_pose(
                        &self.openxr,
                        &session_data,
                        origin.unwrap_or(session_data.current_origin),
                    )
                    .unwrap_or_default();
            }
        }
    }

    pub fn get_controller_pose(
        &self,
        hand: Hand,
        origin: Option<vr::ETrackingUniverseOrigin>,
    ) -> Option<vr::TrackedDevicePose_t> {
        let session_data = self.openxr.session_data.get();
        let controller_index = session_data
            .input_data
            .devices
            .read()
            .unwrap()
            .get_controller_index(hand)?;

        self.get_device_pose(controller_index, origin)
    }

    pub fn get_device_pose(
        &self,
        index: vr::TrackedDeviceIndex_t,
        origin: Option<vr::ETrackingUniverseOrigin>,
    ) -> Option<vr::TrackedDevicePose_t> {
        tracy_span!();

        let session_data = self.openxr.session_data.get();
        let devices = session_data.input_data.devices.read().unwrap();

        devices.get_device(index)?.get_pose(
            &self.openxr,
            &session_data,
            origin.unwrap_or(session_data.current_origin),
        )
    }

    pub fn is_device_connected(&self, index: vr::TrackedDeviceIndex_t) -> bool {
        let session_data = self.openxr.session_data.get();
        let devices = session_data.input_data.devices.read().unwrap();

        devices.get_device(index).is_some_and(|d| d.connected)
    }

    pub fn device_index_to_tracked_device_class(
        &self,
        index: vr::TrackedDeviceIndex_t,
    ) -> Option<vr::ETrackedDeviceClass> {
        let session_data = self.openxr.session_data.get();
        let devices = session_data.input_data.devices.read().unwrap();
        let device = devices.get_device(index)?;

        match device.get_type() {
            TrackedDeviceType::Hmd => Some(vr::ETrackedDeviceClass::HMD),
            TrackedDeviceType::Controller { .. } => Some(vr::ETrackedDeviceClass::Controller),
            #[cfg(feature = "monado")]
            TrackedDeviceType::GenericTracker { .. } => {
                Some(vr::ETrackedDeviceClass::GenericTracker)
            }
        }
    }

    pub fn device_index_to_hand(&self, index: vr::TrackedDeviceIndex_t) -> Option<Hand> {
        let session_data = self.openxr.session_data.get();
        let devices = session_data.input_data.devices.read().unwrap();
        let device = devices.get_device(index)?;

        device.get_controller_hand()
    }

    pub fn get_controller_device_index(&self, hand: Hand) -> Option<vr::TrackedDeviceIndex_t> {
        let session_data = self.openxr.session_data.get();
        let devices = session_data.input_data.devices.read().unwrap();

        devices.get_controller_index(hand)
    }

    pub fn get_device_string_tracked_property(
        &self,
        index: vr::TrackedDeviceIndex_t,
        property: vr::ETrackedDeviceProperty,
    ) -> Option<CString> {
        let session_data = self.openxr.session_data.get();
        let devices = session_data.input_data.devices.read().unwrap();
        let device = devices.get_device(index)?;

        device.get_string_property(property).map(|s| s.to_owned())
    }

    pub fn get_device_int_tracked_property(
        &self,
        index: vr::TrackedDeviceIndex_t,
        property: vr::ETrackedDeviceProperty,
    ) -> Option<i32> {
        let session_data = self.openxr.session_data.get();
        let devices = session_data.input_data.devices.read().unwrap();
        let device = devices.get_device(index)?;

        device.get_int_property(property)
    }

    pub fn get_device_uint_tracked_property(
        &self,
        index: vr::TrackedDeviceIndex_t,
        property: vr::ETrackedDeviceProperty,
    ) -> Option<u64> {
        let session_data = self.openxr.session_data.get();
        let devices = session_data.input_data.devices.read().unwrap();
        let device = devices.get_device(index)?;

        device.get_uint_property(property)
    }
}

#[cfg(all(test, feature = "monado"))]
mod tests {
    use crate::input::{profiles::knuckles::Knuckles, tests::Fixture};
    use crate::openxr_data::Hand;
    use openvr as vr;
    use std::time::{Duration, Instant};

    fn fixture() -> Fixture {
        let f = Fixture::new();
        f.load_actions(c"actions.json");
        f.input.openxr.restart_session(); // use the real session before adding devices
        f
    }

    fn frame(f: &Fixture) {
        f.input.openxr.poll_events();
        f.input.frame_start_update();
    }

    fn refresh(f: &Fixture) {
        // Advance the refresh deadline without sleeping in tests.
        f.input
            .openxr
            .session_data
            .get()
            .input_data
            .devices
            .write()
            .unwrap()
            .last_tracker_refresh = Some(Instant::now() - Duration::from_secs(2));
        frame(f);
    }

    fn serial(f: &Fixture, index: u32) -> String {
        f.input
            .get_device_string_tracked_property(
                index,
                vr::ETrackedDeviceProperty::SerialNumber_String,
            )
            .unwrap()
            .to_str()
            .unwrap()
            .to_owned()
    }

    fn events(f: &Fixture) -> Vec<(u32, u32)> {
        std::iter::from_fn(|| f.next_event())
            .map(|e| (e.eventType, e.trackedDeviceIndex))
            .collect()
    }

    #[test]
    fn get_tracker_pose() {
        let mut f = fixture();
        f.set_interaction_profile::<Knuckles>(fakexr::UserPath::LeftHand);
        frame(&f);
        frame(&f);
        fakexr::add_trackers(f.raw_session());
        refresh(&f);
        assert_eq!(
            f.input.device_index_to_tracked_device_class(2),
            Some(vr::ETrackedDeviceClass::GenericTracker)
        );
        let pose = f
            .input
            .get_device_pose(2, Some(vr::ETrackingUniverseOrigin::Seated))
            .unwrap();
        assert!(pose.bPoseIsValid);
    }

    #[test]
    fn get_tracker_serial() {
        let f = fixture();
        fakexr::add_trackers(f.raw_session());
        refresh(&f);
        assert_eq!(serial(&f, 1), "FAKEXR-SERIAL");
    }

    #[test]
    fn tracker_hotplug_without_profile_events_is_throttled() {
        let f = fixture();
        frame(&f); // initially no trackers or controllers
        events(&f);
        fakexr::add_trackers(f.raw_session());
        frame(&f);
        assert_eq!(f.input.device_index_to_tracked_device_class(1), None);

        refresh(&f);
        assert_eq!(serial(&f, 1), "FAKEXR-SERIAL");
        assert_eq!(
            events(&f),
            vec![(vr::EVREventType::TrackedDeviceActivated as u32, 1)]
        );
        refresh(&f);
        assert!(
            events(&f).is_empty(),
            "known trackers must not be reactivated"
        );
        assert_eq!(f.input.device_index_to_tracked_device_class(2), None);
    }

    #[test]
    fn tracker_slots_survive_reordering_removal_and_reappearance() {
        let f = fixture();
        fakexr::set_tracker_serials(f.raw_session(), &["first"]);
        refresh(&f);
        events(&f);
        fakexr::set_tracker_serials(f.raw_session(), &["second", "first"]);
        refresh(&f);
        assert_eq!(serial(&f, 1), "first");
        assert_eq!(serial(&f, 2), "second");
        assert_eq!(
            events(&f),
            vec![(vr::EVREventType::TrackedDeviceActivated as u32, 2)]
        );

        fakexr::set_tracker_serials(f.raw_session(), &["second"]);
        refresh(&f);
        assert!(!f.input.is_device_connected(1));
        assert_eq!(
            events(&f),
            vec![(vr::EVREventType::TrackedDeviceDeactivated as u32, 1)]
        );
        fakexr::set_tracker_serials(f.raw_session(), &["second", "first"]);
        refresh(&f);
        assert_eq!(serial(&f, 1), "first");
        assert_eq!(serial(&f, 2), "second");
        assert_eq!(
            events(&f),
            vec![(vr::EVREventType::TrackedDeviceActivated as u32, 1)]
        );
    }

    #[test]
    fn trackers_before_controllers_keep_slots_across_session_restart() {
        let mut f = fixture();
        fakexr::set_tracker_serials(f.raw_session(), &["first"]);
        refresh(&f);
        f.set_interaction_profile::<Knuckles>(fakexr::UserPath::LeftHand);
        frame(&f);
        frame(&f);
        assert_eq!(f.controller_slot(Hand::Left), Some((2, true)));
        fakexr::set_tracker_serials(f.raw_session(), &["second", "first"]);
        refresh(&f);
        events(&f);

        f.input.openxr.restart_session();
        assert_eq!(serial(&f, 1), "first");
        assert_eq!(f.controller_slot(Hand::Left), Some((2, true)));
        assert_eq!(serial(&f, 3), "second");
        assert!(events(&f).is_empty(), "no reconnect for surviving devices");
        assert!(
            f.input
                .get_device_pose(1, Some(vr::ETrackingUniverseOrigin::Seated))
                .unwrap()
                .bPoseIsValid
        );
        assert!(
            f.input
                .get_device_pose(3, Some(vr::ETrackingUniverseOrigin::Seated))
                .unwrap()
                .bPoseIsValid
        );
    }

    #[test]
    fn unavailable_tracker_is_deactivated_and_returns_in_the_same_slot() {
        use openxr::SpaceLocationFlags as Flags;
        let f = fixture();
        fakexr::add_trackers(f.raw_session());
        refresh(&f);
        events(&f);

        // The runtime keeps the device listed after power-off but invalidates its pose.
        fakexr::set_tracker_location_flags(f.raw_session(), "FAKEXR-SERIAL", Flags::EMPTY);
        frame(&f);
        assert!(!f.input.is_device_connected(1));
        let pose = f
            .input
            .get_device_pose(1, Some(vr::ETrackingUniverseOrigin::Seated))
            .unwrap();
        assert!(!pose.bDeviceIsConnected);
        assert!(!pose.bPoseIsValid);
        assert_eq!(
            events(&f),
            vec![(vr::EVREventType::TrackedDeviceDeactivated as u32, 1)]
        );
        frame(&f);
        refresh(&f); // enumeration must not reconnect a listed but unavailable device
        assert!(!f.input.is_device_connected(1));
        assert!(events(&f).is_empty());

        fakexr::set_tracker_location_flags(
            f.raw_session(),
            "FAKEXR-SERIAL",
            Flags::POSITION_VALID
                | Flags::ORIENTATION_VALID
                | Flags::POSITION_TRACKED
                | Flags::ORIENTATION_TRACKED,
        );
        frame(&f);
        assert!(f.input.is_device_connected(1));
        assert_eq!(serial(&f, 1), "FAKEXR-SERIAL");
        assert_eq!(
            events(&f),
            vec![(vr::EVREventType::TrackedDeviceActivated as u32, 1)]
        );
    }

    #[test]
    fn held_tracker_pose_remains_connected_but_reports_out_of_range() {
        use openxr::SpaceLocationFlags as Flags;
        let f = fixture();
        fakexr::add_trackers(f.raw_session());
        refresh(&f);
        events(&f);
        fakexr::set_tracker_location_flags(
            f.raw_session(),
            "FAKEXR-SERIAL",
            Flags::POSITION_VALID | Flags::ORIENTATION_VALID,
        );
        frame(&f);
        assert!(f.input.is_device_connected(1));
        let pose = f
            .input
            .get_device_pose(1, Some(vr::ETrackingUniverseOrigin::Seated))
            .unwrap();
        assert!(pose.bDeviceIsConnected);
        assert!(pose.bPoseIsValid);
        assert_eq!(
            pose.eTrackingResult,
            vr::ETrackingResult::Running_OutOfRange
        );
        assert!(events(&f).is_empty());
    }

    #[test]
    fn initially_unavailable_tracker_is_not_announced_as_connected() {
        let f = fixture();
        events(&f);
        fakexr::add_trackers(f.raw_session());
        fakexr::set_tracker_location_flags(
            f.raw_session(),
            "FAKEXR-SERIAL",
            openxr::SpaceLocationFlags::EMPTY,
        );
        refresh(&f);
        assert!(!f.input.is_device_connected(1));
        assert!(events(&f).is_empty());
    }

    #[test]
    fn tracker_hotplug_respects_openvr_capacity() {
        let f = fixture();
        let serials: Vec<String> = (0..vr::k_unMaxTrackedDeviceCount + 2)
            .map(|i| format!("tracker-{i}"))
            .collect();
        let refs: Vec<&str> = serials.iter().map(String::as_str).collect();
        fakexr::set_tracker_serials(f.raw_session(), &refs);
        refresh(&f);
        refresh(&f);
        assert_eq!(
            f.input
                .openxr
                .session_data
                .get()
                .input_data
                .devices
                .read()
                .unwrap()
                .iter()
                .count(),
            vr::k_unMaxTrackedDeviceCount as usize
        );
    }
}
