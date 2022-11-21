mod with_std;
pub use with_std::*;

#[cfg(feature = "with-nalgebra")]
mod with_nalgebra;
#[cfg(feature = "with-nalgebra")]
pub use with_nalgebra::*;

#[cfg(feature = "with-geo")]
mod with_geo;
#[cfg(feature = "with-geo")]
pub use with_geo::*;

#[cfg(feature = "with-nalgebra")]
pub mod close_point_pair_2d;
#[cfg(feature = "with-nalgebra")]
pub use close_point_pair_2d::enumerate_close_pairs_2d;
