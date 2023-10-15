use kiss3d::nalgebra as na30;
use nalgebra as na32;

pub fn p32_to_p30<T>(p: na32::Point3<T>) -> na30::Point3<T>
where
    T: na32::Scalar,
{
    let p: [T; 3] = p.into();
    p.into()
}

pub fn p30_to_p32<T>(p: na30::Point3<T>) -> na32::Point3<T>
where
    T: na30::Scalar,
{
    let p: [T; 3] = p.into();
    p.into()
}

pub fn p30_to_p32_vec<T>(v: &[na30::Point3<T>]) -> Vec<na32::Point3<T>>
where
    T: na30::Scalar,
{
    v.iter().cloned().map(p30_to_p32).collect()
}

// pub fn p32_to_p30_vec<T>(v: &[na32::Point3<T>]) -> Vec<na30::Point3<T>>
// where
//     T: na32::Scalar,
// {
//     v.iter().cloned().map(p32_to_p30).collect()
// }

// pub fn isometry3_30_to_32<T>(pose: na30::Isometry3<T>) -> na32::Isometry3<T>
// where
//     T: na30::Scalar + na32::Scalar + na30::SimdValue + na32::SimdValue + na32::RealField + Copy,
// {
//     let na30::Isometry3 {
//         rotation,
//         translation,
//     } = pose;
//     let na30::coordinates::XYZ { x, y, z } = *translation;
//     let na30::coordinates::IJKW { i, j, k, w } = **rotation;

//     na32::Isometry3 {
//         rotation: na32::UnitQuaternion::from_quaternion(na32::Quaternion::new(w, i, j, k)),
//         translation: na32::Translation3::new(x, y, z),
//     }
// }

pub fn isometry3_32_to_30<T>(pose: na32::Isometry3<T>) -> na30::Isometry3<T>
where
    T: na30::Scalar + na32::Scalar + na30::SimdValue + na32::SimdValue + na30::RealField + Copy,
{
    let na32::Isometry3 {
        rotation,
        translation,
    } = pose;
    let na32::coordinates::XYZ { x, y, z } = *translation;
    let na32::coordinates::IJKW { i, j, k, w } = **rotation;

    na30::Isometry3 {
        rotation: na30::UnitQuaternion::from_quaternion(na30::Quaternion::new(w, i, j, k)),
        translation: na30::Translation3::new(x, y, z),
    }
}
