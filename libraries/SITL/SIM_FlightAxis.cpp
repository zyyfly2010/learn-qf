/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simulator connector for FlightAxis
*/

#include "SIM_FlightAxis.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

#define FLIGHTAXIS_SERVER_IP "192.168.2.48"
#define FLIGHTAXIS_SERVER_PORT 18083

namespace SITL {

// the asprintf() calls are not worth checking for SITL
#pragma GCC diagnostic ignored "-Wunused-result"

FlightAxis::FlightAxis(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str)
{
    last_time_us = get_wall_time_us();
    use_time_sync = false;
    rate_hz = 250 / target_speedup;
}

/*
  extremely primitive SOAP parser that assumes the format used by FlightAxis
*/
void FlightAxis::parse_reply(const char *reply)
{
    for (uint16_t i=0; i<num_keys; i++) {
        const char *p = strstr(reply, keytable[i].key);
        if (p == nullptr) {
            printf("Failed to find key %s\n", keytable[i].key);
        }
        p += strlen(keytable[i].key) + 1;
        keytable[i].ref = atof(p);
        // this assumes key order
        reply = p;
    }
}


/*
  make a SOAP request, returning body of reply
 */
char *FlightAxis::soap_request(const char *action, const char *fmt, ...)
{
    va_list ap;
    char *req1;
    
    va_start(ap, fmt);
    vasprintf(&req1, fmt, ap);
    va_end(ap);

    //printf("%s\n", req1);
    
    // open SOAP socket to FlightAxis
    SocketAPM sock(false);
    if (!sock.connect(FLIGHTAXIS_SERVER_IP, FLIGHTAXIS_SERVER_PORT)) {
        free(req1);
        return nullptr;
    }
    sock.set_blocking(false);

    char *req;
    asprintf(&req, R"(POST / HTTP/1.1
soapaction: '%s'
content-length: %u
content-type: text/xml;charset='UTF-8'
Connection: Keep-Alive

%s)",
             action,
             (unsigned)strlen(req1), req1);
    sock.send(req, strlen(req));
    free(req1);
    free(req);
    char reply[10000];
    memset(reply, 0, sizeof(reply));
    ssize_t ret = sock.recv(reply, sizeof(reply)-1, 1000);
    if (ret <= 0) {
        printf("No data\n");
        return nullptr;
    }
    char *p = strstr(reply, "Content-Length: ");
    if (p == nullptr) {
        printf("No Content-Length\n");
        return nullptr;
    }

    // get the content length
    uint32_t content_length = strtoul(p+16, NULL, 10);
    char *body = strstr(p, "\r\n\r\n");
    if (body == nullptr) {
        printf("No body\n");
        return nullptr;
    }
    body += 4;

    // get the rest of the body
    uint32_t expected_length = content_length + (body - reply);
    if (expected_length >= sizeof(reply)) {
        printf("Reply too large %u\n", expected_length);
        return nullptr;
    }
    while (ret < expected_length) {
        ssize_t ret2 = sock.recv(&reply[ret], sizeof(reply)-(1+ret), 100);
        if (ret2 <= 0) {
            return nullptr;
        }
        ret += ret2;
    }
    return strdup(reply);
}
    


void FlightAxis::exchange_data(const struct sitl_input &input)
{
    if (!controller_started) {
        printf("Starting controller\n");
        // call a restore first. This allows us to connect after the aircraft is changed in RealFlight
        char *reply = soap_request("RestoreOriginalControllerDevice", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<RestoreOriginalControllerDevice><a>1</a><b>2</b></RestoreOriginalControllerDevice>
</soap:Body>
</soap:Envelope>)");
        free(reply);
        reply = soap_request("InjectUAVControllerInterface", R"(<?xml version='1.0' encoding='UTF-8'?>
<soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<InjectUAVControllerInterface><a>1</a><b>2</b></InjectUAVControllerInterface>
</soap:Body>
</soap:Envelope>)");
        free(reply);
        controller_started = true;
    }

    float scaled_servos[8];
    for (uint8_t i=0; i<8; i++) {
        scaled_servos[i] = (input.servos[i] - 1000) / 1000.0f;
    }
    char *reply = soap_request("ExchangeData", R"(<?xml version='1.0' encoding='UTF-8'?><soap:Envelope xmlns:soap='http://schemas.xmlsoap.org/soap/envelope/' xmlns:xsd='http://www.w3.org/2001/XMLSchema' xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance'>
<soap:Body>
<ExchangeData>
<pControlInputs>
<m-selectedChannels>255</m-selectedChannels>
<m-channelValues-0to1>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
<item>%.4f</item>
</m-channelValues-0to1>
</pControlInputs>
</ExchangeData>
</soap:Body>
</soap:Envelope>)",
                               scaled_servos[0],
                               scaled_servos[1],
                               scaled_servos[2],
                               scaled_servos[3],
                               scaled_servos[4],
                               scaled_servos[5],
                               scaled_servos[6],
                               scaled_servos[7]);

    if (reply) {
        parse_reply(reply);
        free(reply);
    }
}
    
    
/*
  update the FlightAxis simulation by one time step
 */
void FlightAxis::update(const struct sitl_input &input)
{
    Vector3f last_velocity_ef = velocity_ef;
    Vector3f last_position = position;
    
    exchange_data(input);

    uint64_t now = get_wall_time_us();
    uint64_t dt = (now - last_time_us) * target_speedup;
    float dt_seconds = dt * 1.0e-6f;
    
    dcm.from_euler(radians(state.m_roll_DEG),
                   radians(state.m_inclination_DEG),
                   -radians(state.m_azimuth_DEG));
    gyro = Vector3f(radians(state.m_rollRate_DEGpSEC),
                    radians(state.m_pitchRate_DEGpSEC),
                    -radians(state.m_yawRate_DEGpSEC)) * target_speedup;
    velocity_ef = Vector3f(state.m_velocityWorldU_MPS,
                             state.m_velocityWorldV_MPS,
                             state.m_velocityWorldW_MPS);
    position = Vector3f(state.m_aircraftPositionY_MTR,
                        state.m_aircraftPositionX_MTR,
                        -state.m_altitudeAGL_MTR);

    // the accel values given in the state are very strange. Calculate
    // it from delta-velocity instead, although this does introduce
    // noise
    Vector3f accel_ef = (velocity_ef - last_velocity_ef) / dt_seconds;
    accel_ef.z -= GRAVITY_MSS;
    accel_body = dcm.transposed() * accel_ef;

    airspeed = state.m_airspeed_MPS;

    update_position();
    time_now_us += dt;

#if 0
    static unsigned counter;
    if (counter++ > 1000) {
        printf("dt=%u\n", (unsigned)dt);
        Vector3f vel2 = (position - last_position) / dt_seconds;
        printf("V1(%.3f,%.3f,%.3f) V2(%.3f,%.3f,%.3f)\n",
               velocity_ef.x, velocity_ef.y, velocity_ef.z,
               vel2.x, vel2.y, vel2.z);
        counter = 0;
    }
#endif

    //velocity_ef = vel2;
    last_time_us = now;
}

} // namespace SITL
