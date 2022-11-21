use anyhow::Result;
use geo::prelude::*;
use itertools::chain;
use log::warn;
use sfcgal::{CoordSeq, ToCoordinates, ToSFCGAL};

/// Computes the IoU among a rectangle and a polygon.
pub fn rect_hull_iou(rect: &geo::Rect<f64>, hull: &geo::Polygon<f64>) -> Result<f64> {
    let intersection = match rect_hull_intersection(rect, hull)? {
        Some(intersection) => intersection,
        None => return Ok(0.0),
    };
    let rect_area = rect.unsigned_area();
    let hull_area = hull.unsigned_area();
    let intersection_area = intersection.unsigned_area();

    let roi = if rect_area <= 1e-6 || hull_area <= 1e-6 {
        0.0
    } else {
        intersection_area * (rect_area.recip() + hull_area.recip())
    };

    Ok(roi)
}

/// Computes the intersection polygon among a rectangle and a polygon.
pub fn rect_hull_intersection(
    rect: &geo::Rect<f64>,
    hull: &geo::Polygon<f64>,
) -> Result<Option<geo::Polygon<f64>>> {
    let rect = {
        let ll = rect.min();
        let ru = rect.max();
        let lu = geo::Coordinate { x: ll.x, y: ru.y };
        let rl = geo::Coordinate { x: ru.x, y: ll.y };
        let exterior = vec![ll.x_y(), rl.x_y(), ru.x_y(), lu.x_y(), ll.x_y()];
        CoordSeq::Polygon(vec![exterior]).to_sfcgal()?
    };
    let hull = {
        let exterior: Vec<_> = hull.exterior().points().map(|point| point.x_y()).collect();
        let interiors = hull.interiors().iter().map(|linestring| {
            let points: Vec<_> = linestring.points().map(|point| point.x_y()).collect();
            points
        });
        let linestrings: Vec<_> = chain!([exterior], interiors).collect();
        let polygon = CoordSeq::Polygon(linestrings).to_sfcgal().ok();

        match polygon {
            Some(polygon) => polygon,
            None => {
                warn!("not a valid polygon");
                return Ok(None);
            }
        }
    };
    let intersection = {
        let intersection = rect.intersection(&hull).ok();
        let intersection = match intersection {
            Some(int) => int,
            None => {
                warn!("failed to compute polygon intersection");
                return Ok(None);
            }
        };
        intersection.to_coordinates::<(f64, f64)>()?
    };
    let polygon_opt = match intersection {
        CoordSeq::Polygon(linestrings) => {
            let mut linestrings_iter = linestrings.into_iter();
            let exterior = geo::LineString(
                linestrings_iter
                    .next()
                    .unwrap()
                    .into_iter()
                    .map(|(x, y)| geo::Coordinate { x, y })
                    .collect(),
            );
            let interiors: Vec<_> = linestrings_iter
                .map(|linestring| {
                    geo::LineString(
                        linestring
                            .into_iter()
                            .map(|(x, y)| geo::Coordinate { x, y })
                            .collect(),
                    )
                })
                .collect();
            Some(geo::Polygon::new(exterior, interiors))
        }
        CoordSeq::Triangle(linestring) => {
            let exterior = geo::LineString(
                linestring
                    .into_iter()
                    .map(|(x, y)| geo::Coordinate { x, y })
                    .collect(),
            );
            Some(geo::Polygon::new(exterior, vec![]))
        }
        CoordSeq::Geometrycollection(collection) => {
            assert!(collection.is_empty());
            None
        }
        _ => {
            warn!("unexpected intersection");
            None
        }
    };
    Ok(polygon_opt)
}
