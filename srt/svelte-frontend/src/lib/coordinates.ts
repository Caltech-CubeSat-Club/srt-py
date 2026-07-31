//Greg Miller (gmiller@gregmiller.net) 2021
//Released as public domain
//http://www.celestialprogramming.com/

let PI = Math.PI;
import * as THREE from 'three';
import { SKY_DOME_RADIUS } from './constants';

//All input and output angles are in radians, jd is Julian Date in UTC
export function raDecToAzEl(ra: number, dec: number, lat: number, lon: number, jd_ut: number): [az: number, el: number, lst: number, HA: number] {
    //Meeus 13.5 and 13.6, modified so West longitudes are negative and 0 is North
    const gmst = greenwichMeanSiderealTime(jd_ut);
    let localSiderealTime = (gmst + lon) % (2 * Math.PI);


    let H = (localSiderealTime - ra);
    if (H < 0) { H += 2 * Math.PI; }
    if (H > Math.PI) { H = H - 2 * Math.PI; }

    let az = (Math.atan2(Math.sin(H), Math.cos(H) * Math.sin(lat) - Math.tan(dec) * Math.cos(lat)));
    let a = (Math.asin(Math.sin(lat) * Math.sin(dec) + Math.cos(lat) * Math.cos(dec) * Math.cos(H)));
    az -= Math.PI;

    if (az < 0) { az += 2 * Math.PI; }
    return [az, a, localSiderealTime, H];
}

export function greenwichMeanSiderealTime(jd: number) {
    //"Expressions for IAU 2000 precession quantities" N. Capitaine1,P.T.Wallace2, and J. Chapront
    const t = ((jd - 2451545.0)) / 36525.0;

    let gmst = earthRotationAngle(jd) + (0.014506 + 4612.156534 * t + 1.3915817 * t * t - 0.00000044 * t * t * t - 0.000029956 * t * t * t * t - 0.0000000368 * t * t * t * t * t) / 60.0 / 60.0 * Math.PI / 180.0;  //eq 42
    gmst %= 2 * Math.PI;
    if (gmst < 0) gmst += 2 * Math.PI;

    return gmst;
}

export function earthRotationAngle(jd: number) {
    //IERS Technical Note No. 32

    const t = jd - 2451545.0;
    const f = jd % 1.0;

    let theta = 2 * Math.PI * (f + 0.7790572732640 + 0.00273781191135448 * t); //eq 14
    theta %= 2 * Math.PI;
    if (theta < 0) theta += 2 * Math.PI;

    return theta;
}

export function raDecToAzElDegrees(ra: number, dec: number, lat: number, lon: number, jd_ut: number): [az: number, el: number, lst: number, HA: number] {
    const [az, el, lst, HA] = raDecToAzEl(ra * Math.PI / 180.0, dec * Math.PI / 180.0, lat * Math.PI / 180.0, lon * Math.PI / 180.0, jd_ut);
    return [az * 180.0 / Math.PI, el * 180.0 / Math.PI, lst * 180.0 / Math.PI, HA * 180.0 / Math.PI];
}

export function localSiderealTime(jd_ut: number, lon: number): number {
    const gmst = greenwichMeanSiderealTime(jd_ut);
    let localSiderealTime = (gmst + lon) % (2 * Math.PI);
    if (localSiderealTime < 0) { localSiderealTime += 2 * Math.PI; }
    return localSiderealTime;
}

export function azEltoVector3(az_deg: number, el_deg: number, RADIUS: number = SKY_DOME_RADIUS): THREE.Vector3 {
    const azRad = -az_deg * PI / 180; // Azimuth is clockwise
    const elRad = el_deg * PI / 180;

    return new THREE.Vector3(
        RADIUS * Math.cos(elRad) * Math.sin(azRad),
        RADIUS * Math.sin(elRad),
        RADIUS * Math.cos(elRad) * Math.cos(azRad)
    );
}

export function pointToAzEl(point: THREE.Vector3, RADIUS: number = SKY_DOME_RADIUS): [az_deg: number, el_deg: number] {
    const x = point.x;
    const y = point.y;
    const z = point.z;

    let el_rad = Math.asin(y / RADIUS);
    let az_rad = -Math.atan2(x, z); // Azimuth is clockwise

    let az_deg = az_rad * 180 / PI;
    if (az_deg < 0) {
        az_deg += 360;
    }
    let el_deg = el_rad * 180 / PI;

    return [az_deg, el_deg];
}