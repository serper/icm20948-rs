use once_cell::sync::Lazy;

const WMM2025_COF: &str = include_str!("../data/WMM2025.COF");

const WGS84_A: f64 = 6378.137; // km
const WGS84_B: f64 = 6356.7523142; // km
const WGS84_RE: f64 = 6371.2; // km
const WGS84_EPS_SQ: f64 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);

#[derive(Debug)]
struct WmmCoeffs {
    epoch: f64,
    nmax: usize,
    g: Vec<f64>,
    h: Vec<f64>,
    g_dot: Vec<f64>,
    h_dot: Vec<f64>,
}

impl WmmCoeffs {
    fn from_cof(cof: &str) -> Result<Self, String> {
        let mut lines = cof.lines();
        let header = lines.next().ok_or("WMM COF is empty")?;
        let epoch = header
            .split_whitespace()
            .next()
            .ok_or("WMM COF missing epoch")?
            .parse::<f64>()
            .map_err(|_| "WMM COF invalid epoch")?;

        let mut entries: Vec<(usize, usize, f64, f64, f64, f64)> = Vec::new();
        let mut nmax = 0usize;

        for line in lines {
            let line = line.trim();
            if line.is_empty() {
                continue;
            }
            let parts: Vec<&str> = line.split_whitespace().collect();
            if parts.len() < 2 {
                continue;
            }
            if parts[0] == "9999" {
                break;
            }
            if parts.len() < 6 {
                return Err(format!("WMM COF malformed line: {line}"));
            }
            let n = parts[0].parse::<usize>().map_err(|_| "Invalid n")?;
            let m = parts[1].parse::<usize>().map_err(|_| "Invalid m")?;
            let g = parts[2].parse::<f64>().map_err(|_| "Invalid g")?;
            let h = parts[3].parse::<f64>().map_err(|_| "Invalid h")?;
            let gdot = parts[4].parse::<f64>().map_err(|_| "Invalid gdot")?;
            let hdot = parts[5].parse::<f64>().map_err(|_| "Invalid hdot")?;
            if n > nmax {
                nmax = n;
            }
            entries.push((n, m, g, h, gdot, hdot));
        }

        let num_terms = (nmax + 1) * (nmax + 2) / 2;
        let mut g = vec![0.0f64; num_terms + 1];
        let mut h = vec![0.0f64; num_terms + 1];
        let mut g_dot = vec![0.0f64; num_terms + 1];
        let mut h_dot = vec![0.0f64; num_terms + 1];

        for (n, m, gv, hv, gsv, hsv) in entries {
            let idx = n * (n + 1) / 2 + m;
            g[idx] = gv;
            h[idx] = hv;
            g_dot[idx] = gsv;
            h_dot[idx] = hsv;
        }

        Ok(Self {
            epoch,
            nmax,
            g,
            h,
            g_dot,
            h_dot,
        })
    }
}

static WMM2025: Lazy<WmmCoeffs> = Lazy::new(|| {
    WmmCoeffs::from_cof(WMM2025_COF).expect("Failed to parse WMM2025.COF")
});

fn is_leap_year(year: i32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}

fn days_in_month(year: i32, month: u32) -> Option<u32> {
    match month {
        1 => Some(31),
        2 => Some(if is_leap_year(year) { 29 } else { 28 }),
        3 => Some(31),
        4 => Some(30),
        5 => Some(31),
        6 => Some(30),
        7 => Some(31),
        8 => Some(31),
        9 => Some(30),
        10 => Some(31),
        11 => Some(30),
        12 => Some(31),
        _ => None,
    }
}

fn decimal_year(year: i32, month: u32, day: u32) -> Result<f64, String> {
    let dim = days_in_month(year, month).ok_or("Invalid month")?;
    if day == 0 || day > dim {
        return Err("Invalid day for month".to_string());
    }
    let mut day_of_year = day;
    for m in 1..month {
        day_of_year += days_in_month(year, m).unwrap_or(0);
    }
    let days_in_year = if is_leap_year(year) { 366.0 } else { 365.0 };
    Ok(year as f64 + (day_of_year as f64 - 1.0) / days_in_year)
}

fn pcup_low(p: &mut [f64], dp: &mut [f64], x: f64, nmax: usize) -> bool {
    let num_terms = (nmax + 1) * (nmax + 2) / 2;
    let mut schmidt = vec![0.0f64; num_terms + 1];
    p[0] = 1.0;
    dp[0] = 0.0;

    let z = ((1.0 - x) * (1.0 + x)).sqrt();

    for n in 1..=nmax {
        for m in 0..=n {
            let idx = n * (n + 1) / 2 + m;
            if n == m {
                let idx1 = (n - 1) * n / 2 + m - 1;
                p[idx] = z * p[idx1];
                dp[idx] = z * dp[idx1] + x * p[idx1];
            } else if n == 1 && m == 0 {
                let idx1 = (n - 1) * n / 2 + m;
                p[idx] = x * p[idx1];
                dp[idx] = x * dp[idx1] - z * p[idx1];
            } else if n > 1 {
                let idx1 = (n - 2) * (n - 1) / 2 + m;
                let idx2 = (n - 1) * n / 2 + m;
                if m > n - 2 {
                    p[idx] = x * p[idx2];
                    dp[idx] = x * dp[idx2] - z * p[idx2];
                } else {
                    let k = (((n - 1) * (n - 1) - (m * m)) as f64)
                        / (((2 * n - 1) * (2 * n - 3)) as f64);
                    p[idx] = x * p[idx2] - k * p[idx1];
                    dp[idx] = x * dp[idx2] - z * p[idx2] - k * dp[idx1];
                }
            }
        }
    }

    schmidt[0] = 1.0;
    for n in 1..=nmax {
        let idx = n * (n + 1) / 2;
        let idx1 = (n - 1) * n / 2;
        schmidt[idx] = schmidt[idx1] * (2 * n - 1) as f64 / n as f64;
        for m in 1..=n {
            let idx = n * (n + 1) / 2 + m;
            let idx1 = n * (n + 1) / 2 + m - 1;
            schmidt[idx] = schmidt[idx1]
                * ((((n - m + 1) * if m == 1 { 2 } else { 1 }) as f64) / (n + m) as f64)
                    .sqrt();
        }
    }

    for n in 1..=nmax {
        for m in 0..=n {
            let idx = n * (n + 1) / 2 + m;
            p[idx] *= schmidt[idx];
            dp[idx] = -dp[idx] * schmidt[idx];
        }
    }
    true
}

fn summation_special(
    coeffs: &WmmCoeffs,
    dt: f64,
    rrp: &[f64],
    sin_mlambda1: f64,
    cos_mlambda1: f64,
    phig_rad: f64,
) -> f64 {
    let nmax = coeffs.nmax;
    let mut pcup_s = vec![0.0f64; nmax + 1];
    pcup_s[0] = 1.0;
    let mut schmidt1 = 1.0;
    let sin_phi = phig_rad.sin();
    let mut by = 0.0;

    for n in 1..=nmax {
        let idx = n * (n + 1) / 2 + 1;
        let schmidt2 = schmidt1 * (2 * n - 1) as f64 / n as f64;
        let schmidt3 = schmidt2 * ((2.0 * n as f64) / (n + 1) as f64).sqrt();
        schmidt1 = schmidt2;
        if n == 1 {
            pcup_s[n] = pcup_s[n - 1];
        } else {
            let k = (((n - 1) * (n - 1) - 1) as f64) / (((2 * n - 1) * (2 * n - 3)) as f64);
            pcup_s[n] = sin_phi * pcup_s[n - 1] - k * pcup_s[n - 2];
        }

        let g = coeffs.g[idx] + dt * coeffs.g_dot[idx];
        let h = coeffs.h[idx] + dt * coeffs.h_dot[idx];
        by += rrp[n] * (g * sin_mlambda1 - h * cos_mlambda1) * pcup_s[n] * schmidt3;
    }

    by
}

pub fn magnetic_declination(
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f64,
    year: i32,
    month: u32,
    day: u32,
) -> Result<f64, String> {
    let dec_year = decimal_year(year, month, day)?;
    magnetic_declination_decimal_year(lat_deg, lon_deg, alt_m, dec_year)
}

pub fn magnetic_declination_decimal_year(
    lat_deg: f64,
    lon_deg: f64,
    alt_m: f64,
    dec_year: f64,
) -> Result<f64, String> {
    if !lat_deg.is_finite() || !lon_deg.is_finite() || !alt_m.is_finite() || !dec_year.is_finite()
    {
        return Err("Invalid input".to_string());
    }
    if lat_deg < -90.0 || lat_deg > 90.0 {
        return Err("Latitude out of range".to_string());
    }

    let coeffs = &*WMM2025;
    let nmax = coeffs.nmax;
    let dt = dec_year - coeffs.epoch;

    let alt_km = alt_m / 1000.0;
    let phi_rad = lat_deg.to_radians();
    let sin_phi = phi_rad.sin();
    let cos_phi = phi_rad.cos();

    // Geodetic to spherical
    let rc = WGS84_A / (1.0 - WGS84_EPS_SQ * sin_phi * sin_phi).sqrt();
    let xp = (rc + alt_km) * cos_phi;
    let zp = (rc * (1.0 - WGS84_EPS_SQ) + alt_km) * sin_phi;
    let r = (xp * xp + zp * zp).sqrt();
    let phig_rad = (zp / r).asin();
    let phig_deg = phig_rad.to_degrees();

    let lambda_deg = lon_deg;
    let lambda_rad = lambda_deg.to_radians();

    // Spherical harmonic variables
    let mut rrp = vec![0.0f64; nmax + 1];
    let rr = WGS84_RE / r;
    rrp[0] = rr * rr;
    for n in 1..=nmax {
        rrp[n] = rrp[n - 1] * rr;
    }

    let mut cos_mlambda = vec![0.0f64; nmax + 1];
    let mut sin_mlambda = vec![0.0f64; nmax + 1];
    let cos_l = lambda_rad.cos();
    let sin_l = lambda_rad.sin();
    cos_mlambda[0] = 1.0;
    sin_mlambda[0] = 0.0;
    if nmax >= 1 {
        cos_mlambda[1] = cos_l;
        sin_mlambda[1] = sin_l;
    }
    for m in 2..=nmax {
        cos_mlambda[m] = cos_mlambda[m - 1] * cos_l - sin_mlambda[m - 1] * sin_l;
        sin_mlambda[m] = cos_mlambda[m - 1] * sin_l + sin_mlambda[m - 1] * cos_l;
    }

    // Legendre functions
    let num_terms = (nmax + 1) * (nmax + 2) / 2;
    let mut p = vec![0.0f64; num_terms + 1];
    let mut dp = vec![0.0f64; num_terms + 1];
    if !pcup_low(&mut p, &mut dp, phig_rad.sin(), nmax) {
        return Err("Legendre computation failed".to_string());
    }

    // Summation
    let mut bx = 0.0f64;
    let mut by = 0.0f64;
    let mut bz = 0.0f64;
    for n in 1..=nmax {
        for m in 0..=n {
            let idx = n * (n + 1) / 2 + m;
            let g = coeffs.g[idx] + dt * coeffs.g_dot[idx];
            let h = coeffs.h[idx] + dt * coeffs.h_dot[idx];
            let tmp = g * cos_mlambda[m] + h * sin_mlambda[m];
            bz -= rrp[n] * tmp * (n as f64 + 1.0) * p[idx];
            by += rrp[n]
                * (g * sin_mlambda[m] - h * cos_mlambda[m])
                * (m as f64)
                * p[idx];
            bx -= rrp[n] * tmp * dp[idx];
        }
    }

    let cos_phig = phig_rad.cos();
    if cos_phig.abs() > 1.0e-10 {
        by /= cos_phig;
    } else {
        by = summation_special(
            coeffs,
            dt,
            &rrp,
            sin_mlambda[1],
            cos_mlambda[1],
            phig_rad,
        );
    }

    // Rotate to geodetic coordinates
    let psi = (phig_deg - lat_deg).to_radians();
    let bx_geo = bx * psi.cos() - bz * psi.sin();
    let by_geo = by;
    let decl = by_geo.atan2(bx_geo).to_degrees();

    Ok(decl)
}
