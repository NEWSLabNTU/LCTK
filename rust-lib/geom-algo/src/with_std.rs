use num_traits::Float;
use std::f64;

pub fn haversine<T>((lon_l, lat_l): (T, T), (lon_r, lat_r): (T, T)) -> T
where
    T: Float,
{
    let lon_diff = (lon_l - lon_r).abs();

    let term1 = lat_r.cos() * lon_diff.sin();
    let term2 = lat_l.cos() * lat_r.sin() - lat_l.sin() * lat_r.cos() * lon_diff.cos();
    let term3 = lat_l.sin() * lat_r.sin() + lat_l.cos() * lat_r.cos() * lon_diff.cos();

    let haversine = ((term1.powi(2) + term2.powi(2)).sqrt() / term3).atan();

    let approx = |lhs: T, rhs: T| -> bool {
        let epsilon = T::from(1e-7).unwrap();
        (lhs - rhs).abs() < epsilon
    };

    // check antipodal cases
    if approx(haversine, T::zero()) {
        if approx(lon_l, lon_r) && approx(lat_l, lat_r) {
            T::zero()
        } else {
            T::from(f64::consts::PI).unwrap()
        }
    } else {
        haversine
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;
    use rand::prelude::*;

    #[test]
    fn haversine_test() {
        use std::f64::consts::*;

        let mut rng = rand::thread_rng();

        assert_abs_diff_eq!(super::haversine((0.0, 0.0), (0.0, 0.0)), 0.0);
        assert_abs_diff_eq!(super::haversine((0.0, 0.0), (FRAC_PI_2, 0.0)), FRAC_PI_2);
        assert_abs_diff_eq!(super::haversine((FRAC_PI_2, 0.0), (PI, 0.0)), FRAC_PI_2);
        assert_abs_diff_eq!(super::haversine((0.0, 0.0), (0.0, FRAC_PI_2)), FRAC_PI_2);
        assert_abs_diff_eq!(
            super::haversine((0.0, -FRAC_PI_4), (0.0, FRAC_PI_4)),
            FRAC_PI_2
        );
        assert_abs_diff_eq!(super::haversine((0.0, -FRAC_PI_2), (0.0, FRAC_PI_2)), PI);

        for _ in 0..100 {
            assert_abs_diff_eq!(
                {
                    let lon = rng.gen_range(0.0..=PI);
                    super::haversine((lon, 0.0), (lon + PI, 0.0))
                },
                PI
            );
            assert_abs_diff_eq!(
                {
                    let lon = rng.gen_range(0.0..=PI);
                    let lat = rng.gen_range((-FRAC_PI_2)..=FRAC_PI_2);
                    super::haversine((lon, lat), (lon + PI, -lat))
                },
                PI
            );
        }
    }
}
