use oort_api::prelude::*;

use std::f32;
use std::f64;
use std::fmt::Debug;
use std::ops::Add;
use std::ops::Div;
use std::ops::Mul;
use std::ops::Neg;
use std::ops::Sub;

/// Generic type that lists functions and constants needed in calculations.
/// Default implementations for f32 and f64 are provided.
pub trait FloatType:
    Sized
    + Copy
    + Debug
    + From<i16>
    + PartialEq
    + PartialOrd
    + Neg<Output = Self>
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
{
    fn zero() -> Self;
    fn one() -> Self;
    fn one_third() -> Self;
    fn pi() -> Self;
    fn two_third_pi() -> Self;
    fn sqrt(self) -> Self;
    /// The cubic root function is pow(x, 1/3) accepting negative arguments
    fn cbrt(self) -> Self {
        if self < Self::zero() {
            -(-self).powf(Self::one_third())
        } else {
            self.powf(Self::one_third())
        }
    }
    fn atan(self) -> Self;
    fn acos(self) -> Self;
    fn sin(self) -> Self;
    fn cos(self) -> Self;
    fn abs(self) -> Self;
    fn powf(self, n: Self) -> Self;
}

impl FloatType for f32 {
    fn zero() -> Self {
        0f32
    }

    fn one_third() -> Self {
        1f32 / 3f32
    }

    fn one() -> Self {
        1f32
    }

    fn two_third_pi() -> Self {
        2f32 * f32::consts::FRAC_PI_3
    }

    fn pi() -> Self {
        f32::consts::PI
    }
    fn sqrt(self) -> Self {
        self.sqrt()
    }
    fn atan(self) -> Self {
        self.atan()
    }
    fn acos(self) -> Self {
        self.acos()
    }
    fn sin(self) -> Self {
        self.sin()
    }
    fn cos(self) -> Self {
        self.cos()
    }
    fn abs(self) -> Self {
        self.abs()
    }
    fn powf(self, n: Self) -> Self {
        self.powf(n)
    }
}

impl FloatType for f64 {
    fn zero() -> Self {
        0f64
    }

    fn one_third() -> Self {
        1f64 / 3f64
    }

    fn one() -> Self {
        1f64
    }

    fn two_third_pi() -> Self {
        2f64 * f64::consts::FRAC_PI_3
    }

    fn pi() -> Self {
        f64::consts::PI
    }
    fn sqrt(self) -> Self {
        self.sqrt()
    }
    fn atan(self) -> Self {
        self.atan()
    }
    fn acos(self) -> Self {
        self.acos()
    }
    fn sin(self) -> Self {
        self.sin()
    }
    fn cos(self) -> Self {
        self.cos()
    }
    fn abs(self) -> Self {
        self.abs()
    }
    fn powf(self, n: Self) -> Self {
        self.powf(n)
    }
}

/// Sorted and unique list of roots of an equation.
#[derive(Debug, PartialEq)]
pub enum Roots {
    /// Equation has no roots
    None([f64; 0]),
    /// Equation has one root (or all roots of the equation are the same)
    One([f64; 1]),
    /// Equation has two roots
    Two([f64; 2]),
    /// Equation has three roots
    Three([f64; 3]),
    /// Equation has four roots
    Four([f64; 4]),
}

impl AsRef<[f64]> for Roots {
    fn as_ref(&self) -> &[f64] {
        match *self {
            Roots::None(ref x) => x,
            Roots::One(ref x) => x,
            Roots::Two(ref x) => x,
            Roots::Three(ref x) => x,
            Roots::Four(ref x) => x,
        }
    }
}

impl Roots {
    fn check_new_root(&self, new_root: f64) -> (bool, usize) {
        let mut pos = 0;
        let mut exists = false;

        for x in self.as_ref().iter() {
            if *x == new_root {
                exists = true;
                break;
            }
            if *x > new_root {
                break;
            }
            pos += 1;
        }

        (exists, pos)
    }

    /// Add a new root to existing ones keeping the list of roots ordered and unique.
    pub fn add_new_root(self, new_root: f64) -> Self {
        match self {
            Roots::None(_) => Roots::One([new_root]),
            _ => {
                let (exists, pos) = self.check_new_root(new_root);

                if exists {
                    self
                } else {
                    let old_roots = self.as_ref();
                    match (old_roots.len(), pos) {
                        (1, 0) => Roots::Two([new_root, old_roots[0]]),
                        (1, 1) => Roots::Two([old_roots[0], new_root]),
                        (2, 0) => Roots::Three([new_root, old_roots[0], old_roots[1]]),
                        (2, 1) => Roots::Three([old_roots[0], new_root, old_roots[1]]),
                        (2, 2) => Roots::Three([old_roots[0], old_roots[1], new_root]),
                        (3, 0) => Roots::Four([new_root, old_roots[0], old_roots[1], old_roots[2]]),
                        (3, 1) => Roots::Four([old_roots[0], new_root, old_roots[1], old_roots[2]]),
                        (3, 2) => Roots::Four([old_roots[0], old_roots[1], new_root, old_roots[2]]),
                        (3, 3) => Roots::Four([old_roots[0], old_roots[1], old_roots[2], new_root]),
                        _ => panic!("Cannot add root"),
                    }
                }
            }
        }
    }
}

pub fn find_roots_linear(a1: f64, a0: f64) -> Roots {
    if a1 == 0.0 {
        if a0 == 0.0 {
            Roots::One([0.0])
        } else {
            Roots::None([])
        }
    } else {
        Roots::One([-a0 / a1])
    }
}

pub fn find_roots_quadratic(a2: f64, a1: f64, a0: f64) -> Roots {
    // Handle non-standard cases
    if a2 == 0.0 {
        // a2 = 0; a1*x+a0=0; solve linear equation
        find_roots_linear(a1, a0)
    } else {
        // Rust lacks a simple way to convert an integer constant to generic type F
        let discriminant = a1 * a1 - 4.0 * a2 * a0;
        if discriminant < 0.0 {
            Roots::None([])
        } else {
            let a2x2 = 2.0 * a2;
            if discriminant == 0.0 {
                Roots::One([-a1 / a2x2])
            } else {
                // To improve precision, do not use the smallest divisor.
                // See https://people.csail.mit.edu/bkph/articles/Quadratics.pdf
                let sq = discriminant.sqrt();

                let (same_sign, diff_sign) = if a1 < 0.0 {
                    (-a1 + sq, -a1 - sq)
                } else {
                    (-a1 - sq, -a1 + sq)
                };

                let (x1, x2) = if same_sign.abs() > a2x2.abs() {
                    let a0x2 = 2.0 * a0;
                    if diff_sign.abs() > a2x2.abs() {
                        // 2*a2 is the smallest divisor, do not use it
                        (a0x2 / same_sign, a0x2 / diff_sign)
                    } else {
                        // diff_sign is the smallest divisor, do not use it
                        (a0x2 / same_sign, same_sign / a2x2)
                    }
                } else {
                    // 2*a2 is the greatest divisor, use it
                    (diff_sign / a2x2, same_sign / a2x2)
                };

                // Order roots
                if x1 < x2 {
                    Roots::Two([x1, x2])
                } else {
                    Roots::Two([x2, x1])
                }
            }
        }
    }
}

pub fn find_roots_cubic_normalized(a2: f64, a1: f64, a0: f64) -> Roots {
    let q = (3.0 * a1 - a2 * a2) / 9.0;
    let r = (9.0 * a2 * a1 - 27.0 * a0 - 2.0 * a2 * a2 * a2) / 54.0;
    let q3 = q * q * q;
    let d = q3 + r * r;
    let a2_div_3 = a2 / 3.0;

    if d < 0.0 {
        let phi_3 = (r / (-q3).sqrt()).acos() / 3.0;
        let sqrt_q_2 = 2.0 * (-q).sqrt();

        Roots::One([sqrt_q_2 * phi_3.cos() - a2_div_3])
            .add_new_root(sqrt_q_2 * (phi_3 - (2.0 * f64::consts::FRAC_PI_3)).cos() - a2_div_3)
            .add_new_root(sqrt_q_2 * (phi_3 + (2.0 * f64::consts::FRAC_PI_3)).cos() - a2_div_3)
    } else {
        let sqrt_d = d.sqrt();
        let s = (r + sqrt_d).cbrt();
        let t = (r - sqrt_d).cbrt();

        if s == t {
            if s + t == 0.0 {
                Roots::One([s + t - a2_div_3])
            } else {
                Roots::One([s + t - a2_div_3]).add_new_root(-(s + t) / 2.0 - a2_div_3)
            }
        } else {
            Roots::One([s + t - a2_div_3])
        }
    }
}

pub fn find_roots_quartic_depressed(a2: f64, a1: f64, a0: f64) -> Roots {
    // Handle non-standard cases
    if a1 == 0.0 {
        // a1 = 0; x^4 + a2*x^2 + a0 = 0; solve biquadratic equation
        debug!("Unhandled depressed quartic form: a1 == 0");
        return Roots::None([]);
    } else if a0 == 0.0 {
        // a0 = 0; x^4 + a2*x^2 + a1*x = 0; reduce to normalized cubic and add zero root
        debug!("Unhandled depressed quartic form: a0 == 0");
        return Roots::None([]);
    } else {
        // Solve the auxiliary equation y^3 + (5/2)*a2*y^2 + (2*a2^2-a0)*y + (a2^3/2 - a2*a0/2 - a1^2/8) = 0
        let a2_pow_2 = a2 * a2;
        let a1_div_2 = a1 / 2.0;
        let b2 = a2 * 5.0 / 2.0;
        let b1 = 2.0 * a2_pow_2 - a0;
        let b0 = (a2_pow_2 * a2 - a2 * a0 - a1_div_2 * a1_div_2) / 2.0;

        // At least one root always exists. The last root is the maximal one.
        let resolvent_roots = find_roots_cubic_normalized(b2, b1, b0);
        let y = resolvent_roots.as_ref().iter().last().unwrap();

        let _a2_plus_2y = a2 + 2.0 * *y;
        if _a2_plus_2y > 0.0 {
            let sqrt_a2_plus_2y = _a2_plus_2y.sqrt();
            let q0a = a2 + *y - a1_div_2 / sqrt_a2_plus_2y;
            let q0b = a2 + *y + a1_div_2 / sqrt_a2_plus_2y;

            let mut roots = find_roots_quadratic(1.0, sqrt_a2_plus_2y, q0a);
            for x in find_roots_quadratic(1.0, -sqrt_a2_plus_2y, q0b)
                .as_ref()
                .iter()
            {
                roots = roots.add_new_root(*x);
            }
            roots
        } else {
            Roots::None([])
        }
    }
}

fn find_roots_via_depressed_quartic(
    a4: f64,
    a3: f64,
    a2: f64,
    a1: f64,
    a0: f64,
    pp: f64,
    rr: f64,
    dd: f64,
) -> Roots {
    // Depressed quartic
    // https://en.wikipedia.org/wiki/Quartic_function#Converting_to_a_depressed_quartic

    // a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0 = 0 => y^4 + p*y^2 + q*y + r.
    let a4_pow_2 = a4 * a4;
    let a4_pow_3 = a4_pow_2 * a4;
    let a4_pow_4 = a4_pow_2 * a4_pow_2;
    // Re-use pre-calculated values
    let p = pp / (8.0 * a4_pow_2);
    let q = rr / (8.0 * a4_pow_3);
    let r =
        (dd + 16.0 * a4_pow_2 * (12.0 * a0 * a4 - 3.0 * a1 * a3 + a2 * a2)) / (256.0 * a4_pow_4);

    let mut roots = Roots::None([]);
    for y in find_roots_quartic_depressed(p, q, r).as_ref().iter() {
        roots = roots.add_new_root(*y - a3 / (4.0 * a4));
    }
    roots
}

/// Solves a quartic equation a4*x^4 + a3*x^3 + a2*x^2 + a1*x + a0 = 0.
///
/// Returned roots are ordered.
/// Precision is about 5e-15 for f64, 5e-7 for f32.
/// WARNING: f32 is often not enough to find multiple roots.
///
pub fn find_roots_quartic(a4: f64, a3: f64, a2: f64, a1: f64, a0: f64) -> Roots {
    // Handle non-standard cases
    if a4 == 0.0 {
        // a4 = 0; a3*x^3 + a2*x^2 + a1*x + a0 = 0; solve cubic equation
        debug!("Unhandled quartic form: a4 == 0");
        return Roots::None([]);
    } else if a0 == 0.0 {
        // a0 = 0; x^4 + a2*x^2 + a1*x = 0; reduce to cubic and arrange results
        debug!("Unhandled quartic form: a0 == 0");
        return Roots::None([]);
    } else if a1 == 0.0 && a3 == 0.0 {
        // a1 = 0, a3 =0; a4*x^4 + a2*x^2 + a0 = 0; solve bi-quadratic equation
        debug!("Unhandled quartic form: a1 && a3 == 0");
        return Roots::None([]);
    } else {
        // Discriminant
        // https://en.wikipedia.org/wiki/Quartic_function#Nature_of_the_roots
        // Partially simplifed to keep intermediate values smaller (to minimize rounding errors).
        let discriminant =
            a4 * a0 * a4 * (256.0 * a4 * a0 * a0 + a1 * (144.0 * a2 * a1 - 192.0 * a3 * a0))
                + a4 * a0 * a2 * a2 * (16.0 * a2 * a2 - 80.0 * a3 * a1 - 128.0 * a4 * a0)
                + (a3
                    * a3
                    * (a4 * a0 * (144.0 * a2 * a0 - 6.0 * a1 * a1)
                        + (a0 * (18.0 * a3 * a2 * a1 - 27.0 * a3 * a3 * a0 - 4.0 * a2 * a2 * a2)
                            + a1 * a1 * (a2 * a2 - 4.0 * a3 * a1))))
                + a4 * a1 * a1 * (18.0 * a3 * a2 * a1 - 27.0 * a4 * a1 * a1 - 4.0 * a2 * a2 * a2);

        let pp = 8.0 * a4 * a2 - 3.0 * a3 * a3;
        let rr = a3 * a3 * a3 + 8.0 * a4 * a4 * a1 - 4.0 * a4 * a3 * a2;
        let delta0 = a2 * a2 - 3.0 * a3 * a1 + 12.0 * a4 * a0;
        let dd = 64.0 * a4 * a4 * a4 * a0 - 16.0 * a4 * a4 * a2 * a2 + 16.0 * a4 * a3 * a3 * a2
            - 16.0 * a4 * a4 * a3 * a1
            - 3.0 * a3 * a3 * a3 * a3;

        // Handle special cases
        let double_root = discriminant == 0.0;
        if double_root {
            let triple_root = double_root && delta0 == 0.0;
            let quadruple_root = triple_root && dd == 0.0;
            let no_roots = dd == 0.0 && pp > 0.0 && rr == 0.0;
            if quadruple_root {
                // Wiki: all four roots are equal
                Roots::One([-a3 / (4.0 * a4)])
            } else if triple_root {
                // Wiki: At least three roots are equal to each other
                // x0 is the unique root of the remainder of the Euclidean division of the quartic by its second derivative
                //
                // Solved by SymPy (ra is the desired reminder)
                // a, b, c, d, e = symbols('a,b,c,d,e')
                // f=a*x**4+b*x**3+c*x**2+d*x+e     // Quartic polynom
                // g=6*a*x**2+3*b*x+c               // Second derivative
                // q, r = div(f, g)                 // SymPy only finds the highest power
                // simplify(f-(q*g+r)) == 0         // Verify the first division
                // qa, ra = div(r/a,g/a)            // Workaround to get the second division
                // simplify(f-((q+qa)*g+ra*a)) == 0 // Verify the second division
                // solve(ra,x)
                // ----- yields
                // (−72*a^2*e+10*a*c^2−3*b^2*c)/(9*(8*a^2*d−4*a*b*c+b^3))
                let x0 = (-72.0 * a4 * a4 * a0 + 10.0 * a4 * a2 * a2 - 3.0 * a3 * a3 * a2)
                    / (9.0 * (8.0 * a4 * a4 * a1 - 4.0 * a4 * a3 * a2 + a3 * a3 * a3));
                let roots = Roots::One([x0]);
                roots.add_new_root(-(a3 / a4 + 3.0 * x0))
            } else if no_roots {
                // Wiki: two complex conjugate double roots
                Roots::None([])
            } else {
                find_roots_via_depressed_quartic(a4, a3, a2, a1, a0, pp, rr, dd)
            }
        } else {
            let no_roots = discriminant > 0.0 && (pp > 0.0 || dd > 0.0);
            return if no_roots {
                // Wiki: two pairs of non-real complex conjugate roots
                Roots::None([])
            } else {
                find_roots_via_depressed_quartic(a4, a3, a2, a1, a0, pp, rr, dd)
            };
        }
    }
}
