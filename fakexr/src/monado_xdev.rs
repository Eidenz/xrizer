use crate::XrType;

use super::{Handle, Space, SpaceType, destroy_handle, get_handle, impl_handle};
use openxr_mndx_xdev_space::bindings::XDevIdMNDX;
use openxr_sys as xr;
use std::ffi::{CString, c_char};
use std::sync::Arc;

#[derive(Default)]
pub(super) struct XDevListMNDX {
    generation_number: u64,
    xdevs: Vec<XDev>,
}

pub(super) struct XDev {
    id: openxr_mndx_xdev_space::bindings::XDevIdMNDX,
    can_create_space: bool,
    name: CString,
    serial: CString,
}

impl_handle!(XDevListMNDX, openxr_mndx_xdev_space::bindings::XDevListMNDX);

pub fn add_trackers(session: xr::Session) {
    set_tracker_serials(session, &["FAKEXR-SERIAL"]);
}

/// Change the runtime's device list without sending a hand profile event.
pub fn set_tracker_serials(session: xr::Session, serials: &[&str]) {
    let session = session.to_handle().unwrap();
    let instance = session.instance.upgrade().unwrap();
    *instance.tracker_serials.lock().unwrap() = serials.iter().map(|s| s.to_string()).collect();
}

/// Model a tracker losing tracking or becoming unavailable without disappearing
/// from the runtime's XDev list (as append-only hotplug implementations behave).
pub fn set_tracker_location_flags(
    session: xr::Session,
    serial: &str,
    flags: xr::SpaceLocationFlags,
) {
    let session = session.to_handle().unwrap();
    let instance = session.instance.upgrade().unwrap();
    instance
        .tracker_location_flags
        .lock()
        .unwrap()
        .insert(serial.to_string(), flags);
}

pub(super) extern "system" fn create_x_dev_list_m_n_d_x(
    session: xr::Session,
    _create_info: *const openxr_mndx_xdev_space::bindings::CreateXDevListInfoMNDX,
    xdev_list: *mut openxr_mndx_xdev_space::bindings::XDevListMNDX,
) -> xr::Result {
    let session = get_handle!(session);
    let instance = session.instance.upgrade().unwrap();
    let xdevs = instance
        .tracker_serials
        .lock()
        .unwrap()
        .iter()
        .enumerate()
        .map(|(i, serial)| XDev {
            // Like Monado, each list is a snapshot, with IDs starting at 43.
            id: XDevIdMNDX::from_raw(43 + i as u64),
            can_create_space: true,
            name: c"FAKEXR-TRACKER".to_owned(),
            serial: CString::new(serial.as_str()).unwrap(),
        })
        .collect();
    let list = Arc::new(XDevListMNDX {
        generation_number: 1, // monado always sets this at 1
        xdevs,
    });

    unsafe {
        *xdev_list = list.to_xr();
    }

    xr::Result::SUCCESS
}

pub(super) extern "system" fn get_x_dev_list_generation_number_m_n_d_x(
    xdev_list: openxr_mndx_xdev_space::bindings::XDevListMNDX,
    generation: *mut u64,
) -> xr::Result {
    let list = get_handle!(xdev_list);

    unsafe {
        *generation = list.generation_number;
    }

    xr::Result::SUCCESS
}

pub(super) extern "system" fn enumerate_x_devs_m_n_d_x(
    xdev_list: openxr_mndx_xdev_space::bindings::XDevListMNDX,
    xdev_capacity_input: u32,
    xdev_count_output: *mut u32,
    xdev_ids: *mut openxr_mndx_xdev_space::bindings::XDevIdMNDX,
) -> xr::Result {
    let xdev_list = get_handle!(xdev_list);

    unsafe { *xdev_count_output = xdev_list.xdevs.len() as u32 };
    let capacity = xdev_list.xdevs.len();
    unsafe { *xdev_count_output = capacity as u32 };

    if !xdev_ids.is_null() && xdev_capacity_input > 0 {
        let ids = unsafe { std::slice::from_raw_parts_mut(xdev_ids, xdev_capacity_input as usize) };

        for (i, xdev) in xdev_list.xdevs.iter().enumerate().take(capacity) {
            if i < xdev_capacity_input as usize {
                ids[i] = xdev.id;
            }
        }
    }

    xr::Result::SUCCESS
}

pub(super) extern "system" fn get_x_dev_properties_m_n_d_x(
    xdev_list: openxr_mndx_xdev_space::bindings::XDevListMNDX,
    get_xdev_info: *const openxr_mndx_xdev_space::bindings::GetXDevInfoMNDX,
    properties: *mut openxr_mndx_xdev_space::bindings::XDevPropertiesMNDX,
) -> xr::Result {
    let xdev_list = get_handle!(xdev_list);
    let get_xdev_info = unsafe { get_xdev_info.as_ref() }.unwrap();

    let Some(xdev) = xdev_list
        .xdevs
        .iter()
        .find(|x| x.id == get_xdev_info.dev_id)
    else {
        return xr::Result::ERROR_INDEX_OUT_OF_RANGE;
    };

    let mut name_buf = [0 as c_char; 256];
    let mut serial_buf = [0 as c_char; 256];

    let name_bytes = xdev.name.as_bytes_with_nul();
    let serial_bytes = xdev.serial.as_bytes_with_nul();

    name_buf[..name_bytes.len()]
        .copy_from_slice(&name_bytes.iter().map(|i| *i as c_char).collect::<Vec<_>>());
    serial_buf[..serial_bytes.len()].copy_from_slice(
        &serial_bytes
            .iter()
            .map(|i| *i as c_char)
            .collect::<Vec<_>>(),
    );

    unsafe {
        *properties = openxr_mndx_xdev_space::bindings::XDevPropertiesMNDX {
            ty: openxr_mndx_xdev_space::bindings::XDevPropertiesMNDX::TYPE,
            next: std::ptr::null_mut(),
            can_create_space: xdev.can_create_space.into(),
            name: name_buf,
            serial: serial_buf,
        };
    }

    xr::Result::SUCCESS
}

pub(super) extern "system" fn destroy_x_dev_list_m_n_d_x(
    xdev_list: openxr_mndx_xdev_space::bindings::XDevListMNDX,
) -> xr::Result {
    destroy_handle(xdev_list);
    xr::Result::SUCCESS
}

pub(super) extern "system" fn create_x_dev_space_m_n_d_x(
    session: xr::Session,
    create_info: *const openxr_mndx_xdev_space::bindings::CreateXDevSpaceInfoMNDX,
    space: *mut xr::Space,
) -> xr::Result {
    let s = get_handle!(session);
    let instance = s.instance.upgrade().unwrap();
    unsafe {
        if (*create_info).xdev_id.into_raw() < 43
            || (*create_info).xdev_id.into_raw()
                >= 43 + instance.tracker_serials.lock().unwrap().len() as u64
        {
            return xr::Result::ERROR_INDEX_OUT_OF_RANGE;
        }
    }

    let create_info = unsafe { create_info.as_ref().unwrap() };
    let serial = instance.tracker_serials.lock().unwrap()
        [(create_info.xdev_id.into_raw() - 43) as usize]
        .clone();
    let xdev_space = Arc::new(Space {
        ty: SpaceType::Tracker(serial),
        offset: create_info.offset,
        session: Arc::downgrade(&s),
    });

    unsafe {
        *space = s.add_space(xdev_space);
    }

    xr::Result::SUCCESS
}
