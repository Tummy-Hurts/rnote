// Imports
use super::Line;
use crate::Transformable;
use crate::ext::{DPose2Ext, Vector2Ext};
use crate::shapes::Shapeable;
use kurbo::{PathEl, Shape};
use p2d::bounding_volume::Aabb;
use p2d::glamx::prelude::{DPose2, DRot2};
use p2d::math::Vector2;
use serde::{Deserialize, Serialize};

/// All doc-comments of this file and [ArrowBuilder][crate::builders] rely on the following
/// graphic:
///
/// ```text
///             tip
///             /|\
///            / | \
///           /  |  \
///   lline_tip  |  rline_tip
///              |
/// lline_start  |  rline_start
///           \  |  /
///            \ | /
///             \|/
///             start
/// ```
///
/// Where `lline_{tip,start}`, `tip`, `start` and `rline_{tip,start}` represent a vector of the arrow.

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
#[serde(default, rename = "logicarrow")]
pub struct LogicArrow {
    /// Start of the arrow.
    pub start: Vector2,

    /// Tip of the arow.
    pub tip: Vector2,
}

impl Transformable for LogicArrow {
    fn translate(&mut self, offset: Vector2) {
        self.start += offset;
        self.tip += offset;
    }

    fn rotate(&mut self, angle: f64, center: Vector2) {
        let pose = DPose2::from_rotation_wrt_center(angle, center);
        self.start = pose.transform_point(self.start);
        self.tip = pose.transform_point(self.tip);
    }

    fn scale(&mut self, scale: Vector2) {
        self.start *= scale;
        self.tip *= scale;
    }
}

impl Shapeable for LogicArrow {
    fn bounds(&self) -> Aabb {
        self.internal_compute_bounds(None)
    }

    fn hitboxes(&self) -> Vec<Aabb> {
        let n_splits = super::hitbox_elems_for_shape_len((self.tip - self.start).length());
        self.split(n_splits)
            .into_iter()
            .map(|line| line.bounds())
            .collect()
    }

    fn outline_path(&self) -> kurbo::BezPath {
        self.to_kurbo(None)
    }
}

impl LogicArrow {
    /// The tip lines (rline & lline) default length, is the actual length when
    /// no stroke width is associated with the arrow, if there is, it is used as
    /// the base length for the actual width calculation.
    const TIP_LINES_DEFAULT_LENGTH: f64 = 10.0;

    /// The angle for `rline` and `lline` to the stem of the arrow.
    const TIP_LINES_STEM_OBTUSE_ANGLE: f64 = (13.0 / 16.0) * std::f64::consts::PI;

    /// sin(TIP_LINES_STEM_OBTUSE_ANGLE) and cos(TIP_LINES_STEM_OBTUSE_ANGLE)
    const SIN_OBTUSE_ANGLE: f64 = 0.555570233;
    const COS_OBTUSE_ANGLE: f64 = 0.8314696123;

    /// Creating a new arrow with the given start and tip vectors.
    pub fn new(start: Vector2, tip: Vector2) -> Self {
        Self { start, tip }
    }

    /// Split the stem of the arrow into the given number of lines.
    pub fn split(&self, n_splits: i32) -> Vec<Line> {
        (0..n_splits)
            .map(|i| {
                let sub_start = self
                    .start
                    .lerp(self.tip, f64::from(i) / f64::from(n_splits));
                let sub_end = self
                    .start
                    .lerp(self.tip, f64::from(i + 1) / f64::from(n_splits));

                Line {
                    start: sub_start,
                    end: sub_end,
                }
            })
            .collect::<Vec<Line>>()
    }

    /// Convert to kurbo shape.
    pub fn to_kurbo(&self, stroke_width: Option<f64>) -> kurbo::BezPath {
        let start = self.start.to_kurbo_point();
        let tip = self.tip.to_kurbo_point();
        let lline_tip = self.compute_lline_tip(stroke_width).to_kurbo_point();
        let rline_tip = self.compute_rline_tip(stroke_width).to_kurbo_point();
        let lline_start = self.compute_lline_start(stroke_width).to_kurbo_point();
        let rline_start = self.compute_rline_start(stroke_width).to_kurbo_point();

        let dir = self.compute_stem_direction_vector();
        let normal = dir.perp();
        let tip_lines_length = Self::compute_tip_lines_length(stroke_width);

        let start_l = (self.start
            + normal * (tip_lines_length * Self::SIN_OBTUSE_ANGLE) / 2.0
            + dir * (tip_lines_length * Self::COS_OBTUSE_ANGLE / 2.0))
            .to_kurbo_point();
        let start_r = (self.start - normal * (tip_lines_length * Self::SIN_OBTUSE_ANGLE / 2.0)
            + dir * (tip_lines_length * Self::COS_OBTUSE_ANGLE / 2.0))
            .to_kurbo_point();
        let tip_l = (self.tip + normal * (tip_lines_length * Self::SIN_OBTUSE_ANGLE / 2.0)
            - dir * (tip_lines_length * Self::COS_OBTUSE_ANGLE / 2.0))
            .to_kurbo_point();
        let tip_r = (self.tip
            - normal * (tip_lines_length * Self::SIN_OBTUSE_ANGLE / 2.0)
            - dir * (tip_lines_length * Self::COS_OBTUSE_ANGLE / 2.0))
            .to_kurbo_point();

        let mut bez_path = kurbo::Line::new(start_l, tip_l).to_path(0.25);
        bez_path.extend([PathEl::MoveTo(start_r), PathEl::LineTo(tip_r)]);
        bez_path.extend([
            PathEl::MoveTo(lline_tip),
            PathEl::LineTo(tip),
            PathEl::LineTo(rline_tip),
        ]);
        bez_path.extend([
            PathEl::MoveTo(lline_start),
            PathEl::LineTo(start),
            PathEl::LineTo(rline_start),
        ]);

        bez_path
    }

    /// Compute the `lline` of the arrow tip.
    ///
    /// Optionally add the stroke width to adjust the length of the line.
    pub fn compute_lline_tip(&self, stroke_width: Option<f64>) -> Vector2 {
        let vec_a =
            self.compute_stem_direction_vector() * Self::compute_tip_lines_length(stroke_width);
        DRot2::new(Self::TIP_LINES_STEM_OBTUSE_ANGLE) * vec_a + self.tip
    }

    /// Compute the `rline` of the arrow tip.
    ///
    /// Optionally add the stroke width to adjust the length of the line.
    pub fn compute_rline_tip(&self, stroke_width: Option<f64>) -> Vector2 {
        let vec_b =
            self.compute_stem_direction_vector() * Self::compute_tip_lines_length(stroke_width);
        DRot2::new(-Self::TIP_LINES_STEM_OBTUSE_ANGLE) * vec_b + self.tip
    }

    /// Compute the `lline_start` of the arrow start.
    ///
    /// Optionally add the stroke width to adjust the length of the line.
    pub fn compute_lline_start(&self, stroke_width: Option<f64>) -> Vector2 {
        let vec_a =
            -self.compute_stem_direction_vector() * Self::compute_tip_lines_length(stroke_width);
        DRot2::new(Self::TIP_LINES_STEM_OBTUSE_ANGLE) * vec_a + self.start
    }

    /// Compute the `rline_start` of the arrow start.
    ///
    /// Optionally add the stroke width to adjust the length of the line.
    pub fn compute_rline_start(&self, stroke_width: Option<f64>) -> Vector2 {
        let vec_b =
            -self.compute_stem_direction_vector() * Self::compute_tip_lines_length(stroke_width);
        DRot2::new(-Self::TIP_LINES_STEM_OBTUSE_ANGLE) * vec_b + self.start
    }

    /// Compute the bounds of the arrow in respect to the given stroke width.
    pub fn internal_compute_bounds(&self, stroke_width: Option<f64>) -> Aabb {
        let lline_end = self.compute_lline_tip(stroke_width);
        let rline_end = self.compute_rline_tip(stroke_width);
        let lline_start = self.compute_lline_start(stroke_width);
        let rline_start = self.compute_rline_start(stroke_width);
        Aabb::from_points([
            lline_end,
            rline_end,
            self.start,
            self.tip,
            lline_start,
            rline_start,
        ])
    }

    /// Compute the normalized direction vector from `start` to `tip`.
    fn compute_stem_direction_vector(&self) -> Vector2 {
        let direction_vector = self.tip - self.start;
        if direction_vector.length() == 0.0 {
            Vector2::X
        } else {
            direction_vector / direction_vector.length()
        }
    }

    /// Compute the length of the tip lines.
    ///
    /// Optionally add the stroke width to adjust the length of the line.
    fn compute_tip_lines_length(stroke_width: Option<f64>) -> f64 {
        let factor = stroke_width.unwrap_or(0.0);
        Self::TIP_LINES_DEFAULT_LENGTH * (1.0 + 0.18 * factor)
    }
}
