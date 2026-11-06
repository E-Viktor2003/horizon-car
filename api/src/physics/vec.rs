//! The small vector type the physics is written in. It is plain `f64` maths
//! with no game types, so the tests run without Horizon.

use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl V3 {
    pub const ZERO: V3 = V3 {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub const fn splat(v: f64) -> Self {
        Self { x: v, y: v, z: v }
    }

    pub fn from_array(a: [f64; 3]) -> Self {
        Self::new(a[0], a[1], a[2])
    }

    pub fn to_array(self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    pub fn dot(self, other: V3) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: V3) -> V3 {
        V3::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }

    /// The unit vector, or zero for a vector too short to have a direction.
    pub fn normalized(self) -> V3 {
        let len = self.length();
        if len < 1e-12 { V3::ZERO } else { self / len }
    }

    /// Gram-Schmidt: `self` without its part along the unit vector `axis`.
    pub fn without(self, axis: V3) -> V3 {
        self - axis * self.dot(axis)
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    pub fn lerp(self, other: V3, t: f64) -> V3 {
        self + (other - self) * t
    }
}

impl Add for V3 {
    type Output = V3;
    fn add(self, other: V3) -> V3 {
        V3::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }
}

impl AddAssign for V3 {
    fn add_assign(&mut self, other: V3) {
        *self = *self + other;
    }
}

impl Sub for V3 {
    type Output = V3;
    fn sub(self, other: V3) -> V3 {
        V3::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }
}

impl SubAssign for V3 {
    fn sub_assign(&mut self, other: V3) {
        *self = *self - other;
    }
}

impl Mul<f64> for V3 {
    type Output = V3;
    fn mul(self, k: f64) -> V3 {
        V3::new(self.x * k, self.y * k, self.z * k)
    }
}

impl Mul<V3> for f64 {
    type Output = V3;
    fn mul(self, v: V3) -> V3 {
        v * self
    }
}

impl Div<f64> for V3 {
    type Output = V3;
    fn div(self, k: f64) -> V3 {
        V3::new(self.x / k, self.y / k, self.z / k)
    }
}

impl Neg for V3 {
    type Output = V3;
    fn neg(self) -> V3 {
        V3::new(-self.x, -self.y, -self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_axes_cross_the_way_the_body_is_built() {
        // +x is left, +y up, +z forward: left cross up is forward.
        let left = V3::new(1.0, 0.0, 0.0);
        let up = V3::new(0.0, 1.0, 0.0);
        assert_eq!(left.cross(up), V3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn a_normalized_vector_has_unit_length() {
        let v = V3::new(3.0, 4.0, 12.0).normalized();
        assert!((v.length() - 1.0).abs() < 1e-12);
        assert_eq!(V3::ZERO.normalized(), V3::ZERO);
    }

    #[test]
    fn removing_an_axis_leaves_the_orthogonal_part() {
        let up = V3::new(0.0, 1.0, 0.0);
        assert_eq!(up.without(up), V3::ZERO);
        let v = V3::new(1.0, 2.0, 0.0);
        assert_eq!(v.without(up), V3::new(1.0, 0.0, 0.0));
    }
}
