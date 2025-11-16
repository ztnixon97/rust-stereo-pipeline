/// Internal distortion models used by camera implementations
#[derive(Debug, Clone)]
pub(super) enum DistortionModel {
    None,
    BrownConrady {
        k1: f64,
        k2: f64,
        k3: f64,
        p1: f64,
        p2: f64,
    },
    Fisheye {
        k1: f64,
        k2: f64,
        k3: f64,
        k4: f64,
    },
}

impl DistortionModel {
    /// Apply distortion to normalized image coordinates
    pub(super) fn distort(&self, x_norm: f64, y_norm: f64) -> (f64, f64) {
        match self {
            DistortionModel::None => (x_norm, y_norm),

            DistortionModel::BrownConrady { k1, k2, k3, p1, p2 } => {
                let r2 = x_norm * x_norm + y_norm * y_norm;
                let r4 = r2 * r2;
                let r6 = r4 * r2;

                let radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

                let x_dist = x_norm * radial
                    + 2.0 * p1 * x_norm * y_norm
                    + p2 * (r2 + 2.0 * x_norm * x_norm);

                let y_dist = y_norm * radial
                    + p1 * (r2 + 2.0 * y_norm * y_norm)
                    + 2.0 * p2 * x_norm * y_norm;

                (x_dist, y_dist)
            }

            DistortionModel::Fisheye { k1, k2, k3, k4 } => {
                let r = (x_norm * x_norm + y_norm * y_norm).sqrt();
                if r < 1e-8 {
                    return (x_norm, y_norm);
                }

                let theta = r.atan();
                let theta2 = theta * theta;
                let theta4 = theta2 * theta2;
                let theta6 = theta4 * theta2;
                let theta8 = theta4 * theta4;

                let theta_d = theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
                let scale = theta_d / r;

                (x_norm * scale, y_norm * scale)
            }
        }
    }

    /// Remove distortion from image coordinates using Newton-Raphson iteration

    pub(super) fn undistort(&self, x_dist: f64, y_dist: f64) -> (f64, f64) {
        match self {
            DistortionModel::None => (x_dist, y_dist),
            _ => {
                let mut x = x_dist;
                let mut y = y_dist;

                for _ in 0..10 {
                    let (fx, fy) = self.distort(x, y);
                    let rx = x_dist - fx;
                    let ry = y_dist - fy;

                    if rx.abs() < 1e-8 && ry.abs() < 1e-10 {
                        break;
                    }

                    // Finite-difference Jacobian
                    let eps = 1e-6;
                    let (fx_x, fy_x) = self.distort(x + eps, y);
                    let (fx_y, fy_y) = self.distort(x, y + eps);

                    let j11 = (fx_x - fx) / eps;
                    let j21 = (fy_x - fy) / eps;
                    let j12 = (fx_y - fx) / eps;
                    let j22 = (fy_y - fy) / eps;

                    // Solve J * [dx, dy]^T = [rx, ry]^T
                    let det = j11 * j22 - j12 * j21;
                    if det.abs() < 1e-18 {
                        break; // Degenerate Jacobian, bail
                    }

                    let dx = (j22 * rx - j12 * ry) / det;
                    let dy = (-j21 * rx + j11 * ry) / det;

                    x += dx;
                    y += dy;
                }

                (x, y)
            }
        }
    }
}
#[cfg(test)]
mod tests {
    use super::DistortionModel;

    #[test]
    fn none_round_trip() {
        let m = DistortionModel::None;
        let (x, y) = (0.123, -0.456);
        let (xd, yd) = m.distort(x, y);
        let (xu, yu) = m.undistort(xd, yd);
        assert!((x - xu).abs() < 1e-12);
        assert!((y - yu).abs() < 1e-12);
    }

    #[test]
    fn brown_conrady_round_trip() {
        let m = DistortionModel::BrownConrady {
            k1: -0.1,
            k2: 0.01,
            k3: 0.0,
            p1: 0.001,
            p2: -0.001,
        };
        let (x, y) = (0.2, -0.15);
        let (xd, yd) = m.distort(x, y);
        let (xu, yu) = m.undistort(xd, yd);
        assert!((x - xu).abs() < 1e-6);
        assert!((y - yu).abs() < 1e-6);
    }

    #[test]
    fn fisheye_round_trip() {
        let m = DistortionModel::Fisheye {
            k1: 0.01,
            k2: 0.001,
            k3: 0.0,
            k4: 0.0,
        };
        let (x, y) = (0.3, 0.1);
        let (xd, yd) = m.distort(x, y);
        let (xu, yu) = m.undistort(xd, yd);
        assert!((x - xu).abs() < 1e-6);
        assert!((y - yu).abs() < 1e-6);
    }
}
