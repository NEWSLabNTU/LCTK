use crate::common::*;
use common_types::serde_types::{DevicePath, DeviceTuple, IntoDeviceTuple};
use nalgebra as na;

/// The map that queries coordinate transformation of any two devices.
#[derive(Debug, Clone)]
pub struct CoordTransformMap {
    pub adjacent: HashMap<DeviceTuple, na::Isometry3<f64>>,
}

impl CoordTransformMap {
    /// Creates a map by reading a list of point to point coordinate transformations.
    ///
    /// It builds a transformation graph internally, in which each node is a device,
    /// and each edge is a transformation among two devices. It propagates over the
    /// graph to compute the transformation of any pair of devices. If there are multiple
    /// paths from one device to another device, it verifies their transformations agrees
    /// to each other. Otherwise, it returns error.
    pub fn new<I>(params: I) -> Result<Self>
    where
        I: IntoIterator<Item = (DevicePath, DevicePath, na::Isometry3<f64>)>,
    {
        // check edges are non-cyclic and non-duplicated
        let edges: Vec<_> = params
            .into_iter()
            .map(|(src, tgt, forward_transform)| -> Result<_> {
                let backward_transform = forward_transform.inverse();

                Ok([
                    (src.clone(), tgt.clone(), forward_transform),
                    (tgt, src, backward_transform),
                ])
            })
            .try_collect()?;
        let edges: Vec<_> = edges.into_iter().flatten().collect();

        // build adjacent map
        let adjacent: HashMap<_, HashMap<_, na::Isometry3<f64>>> =
            edges
                .iter()
                .fold(HashMap::new(), |mut edges, (from, to, transform)| {
                    let transforms = match edges.entry(from) {
                        hash_map::Entry::Occupied(entry) => entry.into_mut(),
                        hash_map::Entry::Vacant(entry) => entry.insert(HashMap::new()),
                    };
                    transforms.insert(to, *transform);
                    transforms
                        .entry(from)
                        .or_insert_with(na::Isometry3::identity);
                    edges
                });

        // extend transforms
        let adjacent = {
            let mut adjacent = adjacent;

            loop {
                let tuples: Vec<_> = adjacent
                    .iter()
                    .map(|(&from, transforms)| -> Result<_> {
                        // iteratoe from -> inter
                        let tuples: Vec<_> = transforms
                            .iter()
                            .map(|(&inter, first)| -> Result<_> {
                                // iterate inter -> to
                                let tuples: Vec<_> = adjacent[&inter]
                                    .iter()
                                    .map(|(&to, second)| -> Result<_> {
                                        let (transform, is_new) =
                                            if let Some(&exist) = adjacent[&from].get(&to) {
                                                // if from -> to edge exists, check the similarity
                                                let is_similar = check_isometry_similarity(
                                                    &exist,
                                                    &(second * first),
                                                );
                                                ensure!(is_similar);
                                                (exist, false)
                                            } else {
                                                // if from -> to edge did not exist, insert the transform
                                                (second * first, true)
                                            };
                                        Ok((from, to, transform, is_new))
                                    })
                                    .try_collect()?;

                                Ok(tuples)
                            })
                            .try_collect()?;

                        Ok(tuples)
                    })
                    .try_collect()?;

                let (tuples, is_new_vec) = tuples
                    .into_iter()
                    .flatten()
                    .flatten()
                    .map(|(from, to, transform, is_new)| ((from, to, transform), is_new))
                    .unzip_n_vec();

                let has_new_transform = is_new_vec.into_iter().any(|yes| yes);
                if !has_new_transform {
                    break;
                }

                let new_adjacent: HashMap<_, HashMap<_, _>> = tuples
                    .into_iter()
                    .map(|(from, to, transform)| (from, (to, transform)))
                    .into_group_map()
                    .into_iter()
                    .map(|(from, transforms)| -> Result<_> {
                        let transforms: HashMap<_, na::Isometry3<f64>> =
                            transforms
                                .into_iter()
                                .into_group_map()
                                .into_iter()
                                .map(|(to, transforms)| -> Result<_> {
                                    let mut transforms = transforms.into_iter();
                                    let lhs = transforms.next().unwrap();
                                    ensure!(
                                        transforms.all(|rhs| check_isometry_similarity(&lhs, &rhs))
                                    );
                                    Ok((to, lhs))
                                })
                                .try_collect()?;
                        Ok((from, transforms))
                    })
                    .try_collect()?;

                adjacent = new_adjacent;
            }

            adjacent
        };

        let adjacent: HashMap<_, _> = adjacent
            .into_iter()
            .flat_map(|(src, transforms)| {
                transforms.into_iter().map(|(tgt, transform)| {
                    (
                        DeviceTuple {
                            src: src.clone(),
                            tgt: tgt.clone(),
                        },
                        transform,
                    )
                })
            })
            .collect();

        Ok(Self { adjacent })
    }

    pub fn get(&self, tuple: impl IntoDeviceTuple) -> Option<&na::Isometry3<f64>> {
        let tuple = tuple.into_device_tuple();
        self.adjacent.get(&tuple)
    }

    pub fn adjacent(&self) -> &HashMap<DeviceTuple, na::Isometry3<f64>> {
        &self.adjacent
    }
}

fn check_isometry_similarity(lhs: &na::Isometry3<f64>, rhs: &na::Isometry3<f64>) -> bool {
    let delta = lhs * rhs.inverse();
    abs_diff_eq!(delta.translation.vector.norm(), 0.0, epsilon = 1e-10)
        && abs_diff_eq!(delta.rotation.angle(), 0.0, epsilon = 1e-10)
}
