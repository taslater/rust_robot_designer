use eframe::egui;
use egui::epaint::Shape;
use egui::{pos2, Color32, Stroke, Pos2};
use geo::{polygon, Polygon};
use geo::relate::Relate;


#[derive(Debug, Clone, Copy)]
pub struct Capsule {
    pub id: usize,
    pub radius: f32,
    pub x1: f32,
    pub y1: f32,
    pub x2: f32,
    pub y2: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PointInsideCapsule {
    Outside,
    InsideEndcap1,
    InsideEndcap2,
    InsideBody,
}

fn circles_overlap(x1: f32, y1: f32, r1: f32, x2: f32, y2: f32, r2: f32) -> bool {
    let dx = x2 - x1;
    let dy = y2 - y1;
    let d_squared = dx * dx + dy * dy;
    let r_sum = r1 + r2;
    let r_sum_squared = r_sum * r_sum;
    d_squared < r_sum_squared
}

const OUTLINE_COLOR: Color32 = Color32::from_rgb(0, 0, 0);
const OUTLINE_WIDTH: f32 = 1.0;
impl Capsule {
    pub fn is_inside_at_all(&self, x: f32, y: f32) -> bool {
        let dx = self.x2 - self.x1;
        let dy = self.y2 - self.y1;
        let t = ((x - self.x1) * dx + (y - self.y1) * dy) / (dx * dx + dy * dy);
        let px = self.x1 + t * dx;
        let py = self.y1 + t * dy;
        let d_squared = (px - x) * (px - x) + (py - y) * (py - y);
        d_squared < self.radius * self.radius
    }

    pub fn is_inside_circle1(&self, x: f32, y: f32) -> bool {
        self.is_inside_circle(x, y, self.x1, self.y1)
    }

    pub fn is_inside_circle2(&self, x: f32, y: f32) -> bool {
        self.is_inside_circle(x, y, self.x2, self.y2)
    }

    pub fn is_inside_circle(&self, x: f32, y: f32, circle_x: f32, circle_y: f32) -> bool {
        let d_squared = (circle_x - x) * (circle_x - x) + (circle_y - y) * (circle_y - y);
        d_squared < self.radius * self.radius
    }

    pub fn is_inside_detail(&self, x: f32, y: f32) -> PointInsideCapsule {
        let is_inside_at_all = self.is_inside_at_all(x, y);
        if !is_inside_at_all {
            return PointInsideCapsule::Outside;
        }

        let is_inside_circle1 = self.is_inside_circle1(x, y);
        let is_inside_circle2 = self.is_inside_circle2(x, y);
        if is_inside_circle1 && is_inside_circle2 {
            PointInsideCapsule::InsideBody
        } else if is_inside_circle1 {
            PointInsideCapsule::InsideEndcap1
        } else if is_inside_circle2 {
            PointInsideCapsule::InsideEndcap2
        } else {
            PointInsideCapsule::InsideBody
        }
    }

    fn body_to_polygon(&self) -> Polygon<f32> {
        let dx = self.x2 - self.x1;
        let dy = self.y2 - self.y1;
        let angle = dy.atan2(dx);
        let angle1 = angle + std::f32::consts::FRAC_PI_2;
        let angle2 = angle - std::f32::consts::FRAC_PI_2;
        let x1 = self.x1 + self.radius * angle1.cos();
        let y1 = self.y1 + self.radius * angle1.sin();
        let x2 = self.x2 + self.radius * angle1.cos();
        let y2 = self.y2 + self.radius * angle1.sin();
        let x3 = self.x2 + self.radius * angle2.cos();
        let y3 = self.y2 + self.radius * angle2.sin();
        let x4 = self.x1 + self.radius * angle2.cos();
        let y4 = self.y1 + self.radius * angle2.sin();
        polygon![
            (x: x1, y: y1),
            (x: x2, y: y2),
            (x: x3, y: y3),
            (x: x4, y: y4),
            (x: x1, y: y1),
        ]
    }

    pub fn intersects_capsule(&self, other: &Capsule) -> bool {
        let body_poly1 = self.body_to_polygon();
        let body_poly2 = other.body_to_polygon();
        if body_poly1.relate(&body_poly2).is_overlaps() {
            return true;
        }
        if circles_overlap(self.x1, self.y1, self.radius, other.x1, other.y1, other.radius) {
            return true;
        }
        if circles_overlap(self.x1, self.y1, self.radius, other.x2, other.y2, other.radius) {
            return true;
        }
        if circles_overlap(self.x2, self.y2, self.radius, other.x1, other.y1, other.radius) {
            return true;
        }
        if circles_overlap(self.x2, self.y2, self.radius, other.x2, other.y2, other.radius) {
            return true;
        }
        false
    }

    pub fn get_center(&self) -> Pos2 {
        Pos2 {
            x: (self.x1 + self.x2) / 2.0,
            y: (self.y1 + self.y2) / 2.0,
        }
    }

    pub fn draw(
        &self,
        painter: &egui::Painter,
        circle1_color: Color32,
        circle2_color: Color32,
        body_color: Color32,
    ) {
        painter.add(Shape::line_segment(
            [pos2(self.x1, self.y1), pos2(self.x2, self.y2)],
            Stroke::new(self.radius * 2.0, OUTLINE_COLOR),
        ));
        painter.add(Shape::line_segment(
            [pos2(self.x1, self.y1), pos2(self.x2, self.y2)],
            Stroke::new((self.radius - OUTLINE_WIDTH) * 2.0, body_color),
        ));

        painter.add(Shape::circle_filled(
            pos2(self.x1, self.y1),
            self.radius,
            OUTLINE_COLOR,
        ));
        painter.add(Shape::circle_filled(
            pos2(self.x1, self.y1),
            self.radius - OUTLINE_WIDTH,
            circle1_color,
        ));

        painter.add(Shape::circle_filled(
            pos2(self.x2, self.y2),
            self.radius,
            OUTLINE_COLOR,
        ));
        painter.add(Shape::circle_filled(
            pos2(self.x2, self.y2),
            self.radius - OUTLINE_WIDTH,
            circle2_color,
        ));
    }
}
