//Greg Miller (gmiller@gregmiller.net) 2021
//Released as public domain
//http://www.celestialprogramming.com/

import * as THREE from 'three';
import { SKY_DOME_RADIUS, PI, LATITUDE_DEG, LONGITUDE_DEG } from './constants';

// Horizontal coordinates - observer-relative (azimuth/elevation), and the
// sky-dome Cartesian vectors the renderer actually places things at. Not
// time-dependent.
export class AzEl {
    // $state on the fields themselves, not just wrapping an AzEl instance in
    // $state(...) from outside - Svelte 5 only deep-proxies plain
    // objects/arrays, not arbitrary class instances, so mutating
    // someAzEl.az_deg = ... on an externally-$state-wrapped instance mutates
    // the object but never notifies dependents. Declaring the fields with
    // $state here makes every instance independently reactive regardless of
    // how it's stored (this is what cursorAzEl in ui.svelte.ts relies on).
    az_deg = $state(0);
    el_deg = $state(0);

    constructor(az_deg: number, el_deg: number) {
        this.az_deg = az_deg;
        this.el_deg = el_deg;
    }

    toVector3(radius = SKY_DOME_RADIUS): THREE.Vector3 {
        const azRad = -this.az_deg * PI / 180; // Azimuth is clockwise
        const elRad = this.el_deg * PI / 180;

        return new THREE.Vector3(
            radius * Math.cos(elRad) * Math.sin(azRad),
            radius * Math.sin(elRad),
            radius * Math.cos(elRad) * Math.cos(azRad)
        );
    }

    toArray(radius = SKY_DOME_RADIUS): [number, number, number] {
        return this.toVector3(radius).toArray();
    }

    static fromVector3(point: THREE.Vector3, radius = SKY_DOME_RADIUS): AzEl {
        const el_rad = Math.asin(point.y / radius);
        const az_rad = -Math.atan2(point.x, point.z); // Azimuth is clockwise

        let az_deg = az_rad * 180 / PI;
        if (az_deg < 0) { az_deg += 360; }
        const el_deg = el_rad * 180 / PI;

        return new AzEl(az_deg, el_deg);
    }
}

// Equatorial coordinates - fixed on the celestial sphere, independent of
// observer or time. Resolving to a horizontal (Az/El) position takes an
// explicit jd_ut (Julian Date, UTC): callers decide whether that's the
// current live/custom time (see stores/time.svelte.ts's `timeState.jd_ut`) or an
// arbitrary time to preview, e.g. for a future time-scrubbing UI.
export class RaDec {
    constructor(public ra_deg: number, public dec_deg: number) {}

    toAzEl(jd_ut: number, lat_deg = LATITUDE_DEG, lon_deg = LONGITUDE_DEG): AzEl {
        const [az_deg, el_deg] = raDecToAzElDegrees(this.ra_deg, this.dec_deg, lat_deg, lon_deg, jd_ut);
        return new AzEl(az_deg, el_deg);
    }
}

// Galactic coordinates - also fixed on the celestial sphere, just oriented
// to the Milky Way's own disk/center rather than Earth's equator. Only
// converts to RaDec directly; toAzEl is a convenience that chains through
// it, exactly like RaDec.toAzEl chains through raDecToAzElDegrees.
export class Galactic {
    constructor(public l_deg: number, public b_deg: number) {}

    toRaDec(): RaDec {
        const [ra_deg, dec_deg] = galacticToEquatorialDegrees(this.l_deg, this.b_deg);
        return new RaDec(ra_deg, dec_deg);
    }

    toAzEl(jd_ut: number, lat_deg = LATITUDE_DEG, lon_deg = LONGITUDE_DEG): AzEl {
        return this.toRaDec().toAzEl(jd_ut, lat_deg, lon_deg);
    }
}

// IAU-defined orientation of the Galactic coordinate system relative to
// equatorial coordinates: RA/Dec of the north galactic pole, and the
// galactic longitude of the north celestial pole. Formally defined in B1950
// but these J2000 numbers (as also used by e.g. Astropy's Galactic frame)
// are accurate to a small fraction of a degree, plenty for placing a sky
// texture.
const GALACTIC_POLE_RA_DEG = 192.85948;
const GALACTIC_POLE_DEC_DEG = 27.12825;
const GALACTIC_LON_OF_NCP_DEG = 122.93192;

// All input and output angles are in radians.
export function galacticToEquatorial(l: number, b: number): [ra: number, dec: number] {
    const raNGP = GALACTIC_POLE_RA_DEG * PI / 180;
    const decNGP = GALACTIC_POLE_DEC_DEG * PI / 180;
    const lNCP = GALACTIC_LON_OF_NCP_DEG * PI / 180;
    const dLon = lNCP - l;

    const dec = Math.asin(Math.sin(decNGP) * Math.sin(b) + Math.cos(decNGP) * Math.cos(b) * Math.cos(dLon));

    const y = Math.cos(b) * Math.sin(dLon);
    const x = Math.cos(decNGP) * Math.sin(b) - Math.sin(decNGP) * Math.cos(b) * Math.cos(dLon);
    let ra = Math.atan2(y, x) + raNGP;
    ra %= 2 * PI;
    if (ra < 0) { ra += 2 * PI; }

    return [ra, dec];
}

export function galacticToEquatorialDegrees(l_deg: number, b_deg: number): [ra_deg: number, dec_deg: number] {
    const [ra, dec] = galacticToEquatorial(l_deg * PI / 180, b_deg * PI / 180);
    return [ra * 180 / PI, dec * 180 / PI];
}

//All input and output angles are in radians, jd is Julian Date in UTC
export function raDecToAzEl(ra: number, dec: number, lat: number, lon: number, jd_ut: number): [az: number, el: number, lst: number, HA: number] {
    //Meeus 13.5 and 13.6, modified so West longitudes are negative and 0 is North
    const gmst = greenwichMeanSiderealTime(jd_ut);
    let localSiderealTime = (gmst + lon) % (2 * PI);

    let H = (localSiderealTime - ra);
    if (H < 0) { H += 2 * PI; }
    if (H > PI) { H = H - 2 * PI; }

    let az = (Math.atan2(Math.sin(H), Math.cos(H) * Math.sin(lat) - Math.tan(dec) * Math.cos(lat)));
    let el = (Math.asin(Math.sin(lat) * Math.sin(dec) + Math.cos(lat) * Math.cos(dec) * Math.cos(H)));
    az -= PI;

    if (az < 0) { az += 2 * PI; }
    return [az, el, localSiderealTime, H];
}

export function greenwichMeanSiderealTime(jd: number) {
    //"Expressions for IAU 2000 precession quantities" N. Capitaine1,P.T.Wallace2, and J. Chapront
    const t = ((jd - 2451545.0)) / 36525.0;

    let gmst = earthRotationAngle(jd) + (0.014506 + 4612.156534 * t + 1.3915817 * t * t - 0.00000044 * t * t * t - 0.000029956 * t * t * t * t - 0.0000000368 * t * t * t * t * t) / 60.0 / 60.0 * PI / 180.0;  //eq 42
    gmst %= 2 * PI;
    if (gmst < 0) gmst += 2 * PI;

    return gmst;
}

export function earthRotationAngle(jd: number) {
    //IERS Technical Note No. 32
    const t = jd - 2451545.0;
    const f = jd % 1.0;

    let theta = 2 * PI * (f + 0.7790572732640 + 0.00273781191135448 * t); //eq 14
    theta %= 2 * PI;
    if (theta < 0) theta += 2 * PI;

    return theta;
}

export function raDecToAzElDegrees(ra: number, dec: number, lat: number, lon: number, jd_ut: number): [az: number, el: number, lst: number, HA: number] {
    const [az, el, lst, HA] = raDecToAzEl(ra * PI / 180.0, dec * PI / 180.0, lat * PI / 180.0, lon * PI / 180.0, jd_ut);
    return [az * 180.0 / PI, el * 180.0 / PI, lst * 180.0 / PI, HA * 180.0 / PI];
}

export function localSiderealTime(jd_ut: number, lon: number): number {
    const gmst = greenwichMeanSiderealTime(jd_ut);
    let localSiderealTime = (gmst + lon) % (2 * PI);
    if (localSiderealTime < 0) { localSiderealTime += 2 * PI; }
    return localSiderealTime;
}

export function lst_radians(jd_ut: number, lon_deg = LONGITUDE_DEG): number {
    return localSiderealTime(jd_ut, lon_deg * PI / 180);
}
