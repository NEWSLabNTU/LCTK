use crate::helpers;
use indexmap::IndexSet;
use noisy_float::prelude::*;
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use serde::{de::Error as _, Deserialize, Deserializer, Serialize, Serializer};
use serde_loader::Json5Path;
use std::ops::{Deref, DerefMut};

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Road {
    pub id: usize,
    pub sensor_id: usize,
    pub road_id: usize,
    pub city: String,
    pub road_name: String,
    pub start: String,
    pub end: String,
    pub direction: Direction,
    pub road_lanenum: usize,
    pub detection_lanenum: usize,
    pub road_class: RoadClass,
    pub detection_type: DetectionType,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Sensor {
    pub id: usize,
    pub sensor_id: usize,
    pub sensor_name: String,
    pub sensor_twd97_x: R64,
    pub sensor_twd97_y: R64,
    pub sensor_x: R64,
    pub sensor_y: R64,
    pub location_type: LocationType,
    pub facing_azimuth: R64,
    pub depression_angle: R64,
    pub depression_angle_x_axis: Option<R64>,
    pub sensor_height_in_meters: R64,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Object {
    pub id: usize,
    pub sensor_id: usize,
    pub road_id: usize,
    pub frame_id: usize,
    pub timestamp: String,
    pub object_id: usize,
    pub object_class: String,
    pub object_length: Option<R64>,
    pub object_width: Option<R64>,
    pub object_bearing: Option<R64>,
    pub base_point: String,
    pub base_distance: Option<R64>,
    pub base_angle: Option<R64>,
    pub base_bearing: Option<R64>,
    pub base_twd97_x: Option<R64>,
    pub base_twd97_y: Option<R64>,
    pub raw_x: Option<R64>,
    pub raw_y: Option<R64>,
    pub object_position: Option<PointsPositionTwd97>,
}
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct PointsPositionTwd97 {
    pub tl: (R64, R64),
    pub tc: (R64, R64),
    pub tr: (R64, R64),
    pub rc: (R64, R64),
    pub br: (R64, R64),
    pub bc: (R64, R64),
    pub bl: (R64, R64),
    pub lc: (R64, R64),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum Direction {
    N,
    NE,
    E,
    SE,
    S,
    SW,
    W,
    NW,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, FromPrimitive)]
pub enum RoadClass {
    NationalExpressway = 1, // 國道
    Expressway = 2,         // 快速道路
    UrbanExpressway = 3,    // 市區快速道路
    ProvincialHighway = 4,  // 省道
    CountyHighway = 5,      // 縣道
    CountyRoad = 6,         // 鄉道
    Road = 7,               // 市區一般道路
    Ramp = 8,               // 匝道
}

impl Serialize for RoadClass {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        (*self as usize).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for RoadClass {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let ordinal = usize::deserialize(deserializer)?;
        Self::from_usize(ordinal)
            .ok_or_else(|| D::Error::custom(format!("invalid ordinal number {}", ordinal)))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, FromPrimitive)]
pub enum DetectionType {
    Straight = 1,     // 直線路段
    Curve = 2,        // 彎曲路段
    Intersection = 3, // 路口
}

impl Serialize for DetectionType {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        (*self as usize).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for DetectionType {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let ordinal = usize::deserialize(deserializer)?;
        Self::from_usize(ordinal)
            .ok_or_else(|| D::Error::custom(format!("invalid ordinal number {}", ordinal)))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, FromPrimitive)]
pub enum LocationType {
    Roadside = 1,            // 路側
    DivisionalIsland = 2,    // 道路中央分隔島
    FastSlowLaneDivisor = 3, // 快慢分隔島
    RoadsideEquipment = 4,   // 路側設備
    Other = 5,               // 其他
}

impl Serialize for LocationType {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        (*self as usize).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for LocationType {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        let ordinal = usize::deserialize(deserializer)?;
        Self::from_usize(ordinal)
            .ok_or_else(|| D::Error::custom(format!("invalid ordinal number {}", ordinal)))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct WaysideConfig {
    #[serde(with = "helpers::serde_non_empty_string")]
    pub name: String,
    pub longitude: R64,
    pub latitude: R64,
    pub azimuth: R64,
    pub wayside_info_dir: Json5Path<Sensor>,
    pub road_info_dir: Json5Path<Road>,
    pub sensors: IndexSet<String>,
}

#[derive(Debug, Clone, Eq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct UniqueWaysideConfig(WaysideConfig);

impl PartialEq for UniqueWaysideConfig {
    fn eq(&self, other: &Self) -> bool {
        self.name.eq(&other.name)
    }
}

impl Deref for UniqueWaysideConfig {
    type Target = WaysideConfig;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for UniqueWaysideConfig {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
