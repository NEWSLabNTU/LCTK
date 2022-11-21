use itertools::chain;
use noisy_float::prelude::{N64, R64};
use num_traits::{Float, NumCast};
use std::{
    collections::{BTreeMap, VecDeque},
    marker::PhantomData,
    ops::RangeInclusive,
};

type Color = bool;

pub fn enumerate_close_pairs_2d<T, P, LI, RI>(
    max_distance: T,
    lpoints: LI,
    rpoints: RI,
) -> impl Iterator<Item = Pair<T, P>>
where
    P: Point<T>,
    T: Float,
    LI: IntoIterator<Item = P>,
    RI: IntoIterator<Item = P>,
{
    let mut points: Vec<(Color, P)> = chain!(
        lpoints.into_iter().map(|p| (true, p)),
        rpoints.into_iter().map(|p| (false, p)),
    )
    .collect();
    let max_distance_sq = max_distance.powi(2);

    // sort points by x coorindates
    points.sort_by_cached_key(|(_color, p)| to_r64(p.x()));

    let state = State::<T, P>::new();

    points
        .into_iter()
        .enumerate()
        .scan(state, move |state, (index, (color, point))| {
            let this = point;

            // swap buffer
            let (this_buf, other_buf) = if color {
                (&mut state.t_buf, &mut state.f_buf)
            } else {
                (&mut state.f_buf, &mut state.t_buf)
            };

            // remove points that are too far away from its x coordinate
            let x_min = this.x() - max_distance;
            this_buf.remove_up_to_x(x_min);
            other_buf.remove_up_to_x(x_min);

            // enumerate nearby points
            let y_min = this.y() - max_distance;
            let y_max = this.y() + max_distance;
            let neary_points = other_buf.points_within_y(y_min..=y_max);

            // keep only point pairs within distance
            let pairs: Vec<_> = neary_points
                .filter_map(|other| {
                    let dist_sq = this.distance_square(other);
                    let ok = dist_sq <= max_distance_sq;

                    // returns (l2_distance, true_point, false_point)
                    ok.then(|| {
                        let distance = dist_sq.sqrt();

                        if color {
                            Pair {
                                distance,
                                lp: this.clone(),
                                rp: other.clone(),
                            }
                        } else {
                            Pair {
                                distance,
                                lp: other.clone(),
                                rp: this.clone(),
                            }
                        }
                    })
                })
                .collect();

            // save new point to buffer
            this_buf.push(IndexedPoint { index, point: this });

            Some(pairs)
        })
        .flatten()
}

#[derive(Debug, Clone)]
pub struct Pair<T, P> {
    pub lp: P,
    pub rp: P,
    pub distance: T,
}

struct State<T, P>
where
    P: Point<T>,
    T: Float,
{
    pub t_buf: Buffer<T, P>,
    pub f_buf: Buffer<T, P>,
}

impl<T, P> State<T, P>
where
    P: Point<T>,
    T: Float,
{
    pub fn new() -> Self {
        State {
            t_buf: Buffer::new(),
            f_buf: Buffer::new(),
        }
    }
}

struct Buffer<T, P>
where
    P: Point<T>,
    T: Float,
{
    vec: VecDeque<IndexedPoint<P>>,
    tree: BTreeMap<Key, P>,
    _phantom: PhantomData<T>,
}

impl<T, P> Buffer<T, P>
where
    P: Point<T>,
    T: Float,
{
    pub fn new() -> Self {
        Self {
            vec: VecDeque::new(),
            tree: BTreeMap::new(),
            _phantom: PhantomData,
        }
    }

    pub fn push(&mut self, point: IndexedPoint<P>) {
        if let Some(back) = self.vec.back() {
            debug_assert!(back.point.x() <= point.point.x());
        }
        self.vec.push_back(point.clone());

        let IndexedPoint { index, point } = point;
        self.tree.insert(
            Key {
                y: to_n64(point.y()),
                x: to_n64(point.x()),
                index,
            },
            point,
        );
    }

    pub fn first(&self) -> Option<&P> {
        self.vec.front().map(|p| &p.point)
    }

    pub fn remove_first(&mut self) {
        let IndexedPoint { index, point } = self.vec.pop_front().unwrap();
        let key = Key {
            y: to_n64(point.y()),
            x: to_n64(point.x()),
            index,
        };
        self.tree.remove(&key);
    }

    pub fn remove_up_to_x(&mut self, x: T) {
        while let Some(point) = self.first() {
            if point.x() < x {
                self.remove_first();
            } else {
                break;
            }
        }
    }

    pub fn points_within_y(&self, range: RangeInclusive<T>) -> impl Iterator<Item = &P> {
        let lower = Key {
            y: to_n64(*range.start()),
            x: N64::neg_infinity(),
            index: 0,
        };
        let upper = Key {
            y: to_n64(*range.end()) + f64::epsilon(),
            x: N64::infinity(),
            index: 0,
        };
        self.tree.range(lower..upper).map(|(_, point)| point)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Key {
    pub y: N64,
    pub x: N64,
    pub index: usize,
}

pub trait Point<T>
where
    Self: Clone,
    T: Float,
{
    fn x(&self) -> T;

    fn y(&self) -> T;

    fn distance_square(&self, other: &Self) -> T {
        let lx = self.x();
        let ly = self.y();
        let rx = other.x();
        let ry = other.y();
        (lx - rx).powi(2) + (ly - ry).powi(2)
    }

    fn distance(&self, other: &Self) -> T {
        self.distance_square(other).sqrt()
    }
}

#[derive(Clone)]
struct IndexedPoint<P> {
    pub index: usize,
    pub point: P,
}

fn to_n64<T>(val: T) -> N64
where
    T: Float,
{
    <N64 as NumCast>::from(val).unwrap()
}

fn to_r64<T>(val: T) -> R64
where
    T: Float,
{
    <R64 as NumCast>::from(val).unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    use itertools::{iproduct, izip};
    use noisy_float::prelude::*;
    use rand::prelude::*;
    use std::time::Instant;

    #[derive(Debug, Clone, PartialEq, Eq)]
    struct XY {
        x: R64,
        y: R64,
    }

    impl Point<R64> for XY {
        fn x(&self) -> R64 {
            self.x
        }

        fn y(&self) -> R64 {
            self.y
        }
    }

    #[test]
    fn enumerate_close_pairs_test() {
        let mut rng = rand::thread_rng();
        let max_dist = r64(0.5);

        // generate points
        let lcnt = rng.gen_range(1000..=2000);
        let rcnt = rng.gen_range(1000..=2000);

        let lpoints: Vec<_> = (0..lcnt)
            .map(|_| XY {
                x: r64(rng.gen_range(-1.0..=1.0)),
                y: r64(rng.gen_range(-1.0..=1.0)),
            })
            .collect();
        let rpoints: Vec<_> = (0..rcnt)
            .map(|_| XY {
                x: r64(rng.gen_range(-1.0..=1.0) + 1.0),
                y: r64(rng.gen_range(-1.0..=1.0)),
            })
            .collect();

        let since = Instant::now();
        let mut pairs: Vec<_> =
            enumerate_close_pairs_2d(max_dist, lpoints.clone(), rpoints.clone()).collect();
        eprintln!("elapsed time (implemented): {:?}", since.elapsed());

        let since = Instant::now();
        let mut expects: Vec<_> = iproduct!(lpoints, rpoints)
            .filter_map(|(lp, rp)| {
                let distance = lp.distance(&rp);
                let ok = distance <= max_dist;
                ok.then(|| Pair { distance, lp, rp })
            })
            .collect();
        eprintln!("elapsed time (naive): {:?}", since.elapsed());

        pairs.sort_by_cached_key(|pair| (pair.lp.x(), pair.lp.y(), pair.rp.x(), pair.rp.y()));
        expects.sort_by_cached_key(|pair| (pair.lp.x(), pair.lp.y(), pair.rp.x(), pair.rp.y()));

        izip!(expects, pairs).for_each(|(expect, pair)| {
            assert_eq!(expect.lp, pair.lp);
            assert_eq!(expect.rp, pair.rp);
            assert!((expect.distance - pair.distance).abs() <= 1e-5);
        });
    }
}
