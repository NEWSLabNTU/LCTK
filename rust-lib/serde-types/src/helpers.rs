use indexmap::IndexMap;
use serde::{de::Error as _, ser::Error as _, Deserialize, Deserializer, Serialize, Serializer};
use std::{
    collections::HashMap,
    convert::TryFrom,
    hash::{Hash, Hasher},
    ops::{Bound, Bound::*, RangeInclusive},
};
use url::Url;

pub fn hash_index_map<H, K, V>(map: &IndexMap<K, V>, state: &mut H)
where
    K: Hash,
    V: Hash,
    H: Hasher,
{
    let vec: Vec<_> = map.iter().collect();
    vec.hash(state)
}

pub fn zero_chrono_duration() -> chrono::Duration {
    chrono::Duration::zero()
}

pub fn bool_const<const VALUE: bool>() -> bool {
    VALUE
}

pub fn hash_vec_indexmap_map<H, K, V>(map: &[IndexMap<K, V>], state: &mut H)
where
    K: Hash,
    V: Hash,
    H: Hasher,
{
    map.iter().for_each(|v| hash_index_map(v, state));
}

pub fn empty_hash_map<K, V>() -> HashMap<K, V> {
    HashMap::new()
}

pub fn empty_index_map<K, V>() -> IndexMap<K, V> {
    IndexMap::new()
}

pub fn empty_vec<T>() -> Vec<T> {
    vec![]
}

pub mod serde_non_empty_string {
    use super::*;

    pub fn serialize<S>(text: &str, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        if text.is_empty() {
            return Err(S::Error::custom("string must not be empty"));
        }
        text.serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<String, D::Error>
    where
        D: Deserializer<'de>,
    {
        let text = String::deserialize(deserializer)?;
        if text.is_empty() {
            return Err(D::Error::custom("string must not be empty"));
        }
        Ok(text)
    }
}

pub mod serde_url {
    use super::*;

    pub fn serialize<S>(url: &Url, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        if !["http", "https"].contains(&url.scheme()) {
            return Err(S::Error::custom(
                "the base_url must start with either http or https",
            ));
        }

        url.serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Url, D::Error>
    where
        D: Deserializer<'de>,
    {
        let url = Url::deserialize(deserializer)?;

        if !["http", "https"].contains(&url.scheme()) {
            return Err(D::Error::custom(
                "the base_url must start with either http or https",
            ));
        }

        Ok(url)
    }
}

pub mod serde_fourcc {
    use super::*;

    pub fn serialize<S>(fourcc: &[u8; 4], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        serde_bytes::serialize(fourcc.as_ref(), serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<[u8; 4], D::Error>
    where
        D: Deserializer<'de>,
    {
        let bytes: Vec<u8> = serde_bytes::deserialize(deserializer)?;
        let bytes = <[u8; 4]>::try_from(bytes)
            .map_err(|_| D::Error::custom("fourcc must be specified in exactly 4 bytes"))?;
        Ok(bytes)
    }
}

pub mod serde_chrono_duration {
    use super::*;

    pub fn serialize<S>(duration: &chrono::Duration, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let zero = chrono::Duration::zero();
        let is_negative = duration < &zero;
        let std_duration = if is_negative { -*duration } else { *duration }
            .to_std()
            .map_err(|err| S::Error::custom(format!("invalid duration: {}", err)))?;
        let text = format!(
            "{}{}",
            if is_negative { "-" } else { "" },
            humantime::format_duration(std_duration)
        );
        text.serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<chrono::Duration, D::Error>
    where
        D: Deserializer<'de>,
    {
        let text = String::deserialize(deserializer)?;
        let is_negative = text
            .get(0..1)
            .ok_or_else(|| D::Error::custom("empty string is not allowed"))?
            == "-";
        let duration_text = if is_negative { &text[1..] } else { &text };
        let std_duration = humantime::parse_duration(duration_text).map_err(|err| {
            D::Error::custom(format!("cannot parse duration string '{}': {}", text, err))
        })?;
        let duration = {
            let duration = chrono::Duration::from_std(std_duration).map_err(|err| {
                D::Error::custom(format!("cannot parse duration '{}': {}", text, err))
            })?;
            if is_negative {
                -duration
            } else {
                duration
            }
        };
        Ok(duration)
    }
}

#[cfg(feature = "with-nalgebra")]
pub use with_nalgebra::*;

#[cfg(feature = "with-nalgebra")]
mod with_nalgebra {
    use super::*;
    use nalgebra as na;

    #[derive(Serialize, Deserialize)]
    struct Point3<T> {
        pub x: T,
        pub y: T,
        pub z: T,
    }

    #[derive(Serialize, Deserialize)]
    struct EulerRotation<T> {
        pub roll: T,
        pub pitch: T,
        pub yaw: T,
    }

    #[derive(Serialize, Deserialize)]
    struct EulerIsometry3<T> {
        pub translation: Point3<T>,
        pub rotation: EulerRotation<T>,
    }

    pub mod serde_euler_isometry3 {
        use super::*;

        pub fn serialize<S, T>(rot: &na::Isometry3<T>, serializer: S) -> Result<S::Ok, S::Error>
        where
            T: na::SimdRealField + na::RealField + Serialize,
            T::Element: na::SimdRealField,
            S: Serializer,
        {
            use na::base::coordinates::XYZ;
            let na::Isometry3 {
                translation,
                rotation,
            } = rot;
            let XYZ { x, y, z } = (**translation).clone();
            let (roll, pitch, yaw) = rotation.euler_angles();
            EulerIsometry3 {
                translation: Point3 { x, y, z },
                rotation: EulerRotation { roll, pitch, yaw },
            }
            .serialize(serializer)
        }

        pub fn deserialize<'de, D, T>(deserializer: D) -> Result<na::Isometry3<T>, D::Error>
        where
            T: na::SimdRealField + na::RealField + Deserialize<'de>,
            D: Deserializer<'de>,
        {
            let EulerIsometry3 {
                translation: Point3 { x, y, z },
                rotation: EulerRotation { roll, pitch, yaw },
            } = EulerIsometry3::deserialize(deserializer)?;
            let translation = na::Translation3::new(x, y, z);
            let rotation = na::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
            let isometry = na::Isometry3 {
                translation,
                rotation,
            };
            Ok(isometry)
        }
    }

    pub mod serde_euler_rotation {
        use super::*;

        pub fn serialize<S, T>(
            rot: &na::UnitQuaternion<T>,
            serializer: S,
        ) -> Result<S::Ok, S::Error>
        where
            T: na::SimdRealField + na::RealField + Serialize,
            S: Serializer,
        {
            let (roll, pitch, yaw) = rot.euler_angles();
            EulerRotation { roll, pitch, yaw }.serialize(serializer)
        }

        pub fn deserialize<'de, D, T>(deserializer: D) -> Result<na::UnitQuaternion<T>, D::Error>
        where
            T: na::SimdRealField + na::RealField + Deserialize<'de>,
            D: Deserializer<'de>,
        {
            let EulerRotation { roll, pitch, yaw } = EulerRotation::deserialize(deserializer)?;
            let rot = na::UnitQuaternion::from_euler_angles(roll, pitch, yaw);
            Ok(rot)
        }
    }
}

pub use bound::*;
mod bound {
    use super::*;

    fn unpack<T>(bound: &Bound<T>) -> (Option<&T>, Option<&T>) {
        match bound {
            Unbounded => (None, None),
            Included(val) => (None, Some(val)),
            Excluded(val) => (Some(val), None),
        }
    }

    fn pack<T>(bound: Option<T>, ibound: Option<T>) -> Option<Bound<T>> {
        let output = match (bound, ibound) {
            (None, None) => Unbounded,
            (Some(val), None) => Excluded(val),
            (None, Some(val)) => Included(val),
            (Some(_), Some(_)) => return None,
        };
        Some(output)
    }

    #[derive(Serialize, Deserialize)]
    struct RawBound<T> {
        pub min: Option<T>,
        pub imin: Option<T>,
        pub max: Option<T>,
        pub imax: Option<T>,
    }

    impl<'a, T> RawBound<&'a T> {
        pub fn from_bound((lower, upper): &'a (Bound<T>, Bound<T>)) -> Self {
            let (min, imin) = unpack(lower);
            let (max, imax) = unpack(upper);

            RawBound {
                min,
                imin,
                max,
                imax,
            }
        }
    }

    impl<T> RawBound<T> {
        pub fn into_bound(self) -> Result<(Bound<T>, Bound<T>), &'static str> {
            let RawBound {
                min,
                imin,
                max,
                imax,
            } = self;

            let lower = pack(min, imin).ok_or("min and imin must not be both specified")?;
            let upper = pack(max, imax).ok_or("max and imax must not be both specified")?;

            Ok((lower, upper))
        }
    }

    pub mod serde_bound {
        use super::*;

        pub fn serialize<S, T>(
            bound: &(Bound<T>, Bound<T>),
            serializer: S,
        ) -> Result<S::Ok, S::Error>
        where
            T: Serialize,
            S: Serializer,
        {
            RawBound::from_bound(bound).serialize(serializer)
        }

        pub fn deserialize<'de, D, T>(deserializer: D) -> Result<(Bound<T>, Bound<T>), D::Error>
        where
            T: Deserialize<'de>,
            D: Deserializer<'de>,
        {
            let raw: RawBound<T> = Deserialize::deserialize(deserializer)?;
            raw.into_bound().map_err(D::Error::custom)
        }
    }

    pub mod serde_option_bound {
        use super::*;

        type Range<T> = (Bound<T>, Bound<T>);

        pub fn serialize<S, T>(
            bound: &Option<(Bound<T>, Bound<T>)>,
            serializer: S,
        ) -> Result<S::Ok, S::Error>
        where
            T: Serialize,
            S: Serializer,
        {
            bound
                .as_ref()
                .map(|bound| RawBound::from_bound(bound))
                .serialize(serializer)
        }

        pub fn deserialize<'de, D, T>(deserializer: D) -> Result<Option<Range<T>>, D::Error>
        where
            T: Deserialize<'de>,
            D: Deserializer<'de>,
        {
            let bound = Option::<RawBound<T>>::deserialize(deserializer)?
                .map(|raw| raw.into_bound())
                .transpose()
                .map_err(D::Error::custom)?;
            Ok(bound)
        }
    }

    pub mod serde_range_inclusive {
        use super::*;

        #[derive(Serialize, Deserialize)]
        struct RawBound<T> {
            pub imin: T,
            pub imax: T,
        }

        pub fn serialize<S, T>(range: &RangeInclusive<T>, serializer: S) -> Result<S::Ok, S::Error>
        where
            T: Serialize,
            S: Serializer,
        {
            RawBound {
                imin: range.start(),
                imax: range.end(),
            }
            .serialize(serializer)
        }

        pub fn deserialize<'de, D, T>(deserializer: D) -> Result<RangeInclusive<T>, D::Error>
        where
            T: Deserialize<'de>,
            D: Deserializer<'de>,
        {
            let RawBound { imin, imax } = Deserialize::deserialize(deserializer)?;

            Ok(imin..=imax)
        }
    }
}

#[cfg(feature = "with-measurements")]
pub mod serde_length {
    use super::*;
    use measurements::Length;

    pub fn serialize<S>(len: &Length, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        format!("{:.}m", len.as_meters()).serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Length, D::Error>
    where
        D: Deserializer<'de>,
    {
        let text = String::deserialize(deserializer)?;

        let parse_number = |text: &str| -> Result<f64, D::Error> {
            let value: f64 = text
                .parse()
                .map_err(|_| D::Error::custom(format!("{} is not a valid number", text)))?;
            Ok(value)
        };

        let length = if let Some(prefix) = text.strip_suffix("nm") {
            let value = parse_number(prefix)?;
            Length::from_nanometers(value)
        } else if let Some(prefix) = text.strip_suffix("um") {
            let value = parse_number(prefix)?;
            Length::from_micrometers(value)
        } else if let Some(prefix) = text.strip_suffix("µm") {
            let value = parse_number(prefix)?;
            Length::from_micrometers(value)
        } else if let Some(prefix) = text.strip_suffix("mm") {
            let value = parse_number(prefix)?;
            Length::from_millimeters(value)
        } else if let Some(prefix) = text.strip_suffix("cm") {
            let value = parse_number(prefix)?;
            Length::from_centimeters(value)
        } else if let Some(prefix) = text.strip_suffix("dm") {
            let value = parse_number(prefix)?;
            Length::from_decimeters(value)
        } else if let Some(prefix) = text.strip_suffix("hm") {
            let value = parse_number(prefix)?;
            Length::from_hectometers(value)
        } else if let Some(prefix) = text.strip_suffix("km") {
            let value = parse_number(prefix)?;
            Length::from_kilometers(value)
        } else if let Some(prefix) = text.strip_suffix('m') {
            let value = parse_number(prefix)?;
            Length::from_meters(value)
        } else if let Some(prefix) = text.strip_suffix("in") {
            let value = parse_number(prefix)?;
            Length::from_inches(value)
        } else if let Some(prefix) = text.strip_suffix("yd") {
            let value = parse_number(prefix)?;
            Length::from_yards(value)
        } else if let Some(prefix) = text.strip_suffix("mi") {
            let value = parse_number(prefix)?;
            Length::from_miles(value)
        } else if let Some(prefix) = text.strip_suffix("furlong") {
            let value = parse_number(prefix)?;
            Length::from_furlongs(value)
        } else if let Some(prefix) = text.strip_suffix("ft") {
            let value = parse_number(prefix)?;
            Length::from_feet(value)
        } else {
            return Err(D::Error::custom(
                "Unable to parse '{}' as a length measure.
It must be a floating number plus a length unit, for example, '10.0m'.",
            ));
        };

        Ok(length)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serde_chrono_duration_test() {
        #[derive(Debug, Clone, Serialize, Deserialize)]
        #[serde(transparent)]
        struct Wrapper(#[serde(with = "super::serde_chrono_duration")] chrono::Duration);

        let duration: Wrapper = serde_json::from_str(r#""1m 30s""#).unwrap();
        assert_eq!(duration.0, chrono::Duration::seconds(90));

        let duration: Wrapper = serde_json::from_str(r#""-1m 30s""#).unwrap();
        assert_eq!(duration.0, chrono::Duration::seconds(-90));

        let duration = Wrapper(chrono::Duration::seconds(90));
        assert_eq!(serde_json::to_string(&duration).unwrap(), r#""1m 30s""#);

        let duration = Wrapper(chrono::Duration::seconds(-90));
        assert_eq!(serde_json::to_string(&duration).unwrap(), r#""-1m 30s""#);
    }
}
