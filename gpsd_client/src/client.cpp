#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <libgpsmm.h>

using namespace gps_common;

class GPSDClient {
  public:
    GPSDClient() : privnode("~") {

    }

    bool start() {
      status_pub = node.advertise<GPSStatus>("status", 1);
      fix_pub = node.advertise<GPSFix>("fix", 1);

      std::string host = "localhost";
      int port = 2947;
      privnode.getParam("host", host);
      privnode.getParam("port", port);

      char port_s[12];
      snprintf(port_s, 12, "%d", port);

      gps_data_t *resp = gps.open(host.c_str(), port_s);
      if (resp == NULL) {
        ROS_ERROR("Failed to open GPSd");
        return false;
      }

#if GPSD_API_MAJOR_VERSION == 4
      resp = gps.stream(WATCH_ENABLE);
#elif GPSD_API_MAJOR_VERSION == 3
      gps.query("w\n");
#else
#error "gpsd_client only supports gpsd API versions 3 and 4"
#endif

      ROS_INFO("GPSd opened");
      return true;
    }

    void step() {
      gps_data_t *p = gps.poll();
      process_data(p);
    }

    void stop() {
      // gpsmm doesn't have a close method? OK ...
    }

  private:
    ros::NodeHandle node;
    ros::NodeHandle privnode;
    ros::Publisher status_pub;
    ros::Publisher fix_pub;
    gpsmm gps;

    void process_data(struct gps_data_t* p) {
      if (p == NULL)
        return;

      if (!p->online)
        return;

      ros::Time time = ros::Time::now();

      GPSStatus status;
      GPSFix fix;

      status.header.stamp = time;
      fix.header.stamp = time;

      status.satellites_used = p->satellites_used;

      status.satellite_used_prn.resize(status.satellites_used);
      for (int i = 0; i < status.satellites_used; ++i) {
        status.satellite_used_prn[i] = p->used[i];
      }

#if GPSD_API_MAJOR_VERSION == 4
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 3
#define SATS_VISIBLE p->satellites
#endif

      status.satellites_visible = SATS_VISIBLE;

      status.satellite_visible_prn.resize(status.satellites_visible);
      status.satellite_visible_z.resize(status.satellites_visible);
      status.satellite_visible_azimuth.resize(status.satellites_visible);
      status.satellite_visible_snr.resize(status.satellites_visible);

      for (int i = 0; i < SATS_VISIBLE; ++i) {
        status.satellite_visible_prn[i] = p->PRN[i];
        status.satellite_visible_z[i] = p->elevation[i];
        status.satellite_visible_azimuth[i] = p->azimuth[i];
        status.satellite_visible_snr[i] = p->ss[i];
      }

      if (p->status & STATUS_FIX)
        status.status |= 1; // FIXME: gpsmm puts its constants in the global
                            // namespace, so `GPSStatus::STATUS_FIX' is illegal.

      if (p->status & STATUS_DGPS_FIX)
        status.status |= 3; // same here

      // if (status.status) {
        fix.time = p->fix.time;
        fix.latitude = p->fix.latitude;
        fix.longitude = p->fix.longitude;
        fix.altitude = p->fix.altitude;
        fix.track = p->fix.track;
        fix.speed = p->fix.speed;
        fix.climb = p->fix.climb;

#if GPSD_API_MAJOR_VERSION > 3
        fix.pdop = p->dop.pdop;
        fix.hdop = p->dop.hdop;
        fix.vdop = p->dop.vdop;
        fix.tdop = p->dop.tdop;
        fix.gdop = p->dop.gdop;
#else
        fix.pdop = p->pdop;
        fix.hdop = p->hdop;
        fix.vdop = p->vdop;
        fix.tdop = p->tdop;
        fix.gdop = p->gdop;
#endif

        fix.err = p->epe;
        fix.err_vert = p->fix.epv;
        fix.err_track = p->fix.epd;
        fix.err_speed = p->fix.eps;
        fix.err_climb = p->fix.epc;
        fix.err_time = p->fix.ept;

        /* TODO: attitude */
      // }

      status_pub.publish(status);
      fix_pub.publish(fix);
    }
};

int main(int argc, char ** argv) {
  ros::init(argc, argv, "gpsd_client");

  GPSDClient client;

  if (!client.start())
    return -1;


  while(ros::ok()) {
    ros::spinOnce();
    client.step();
  }

  client.stop();
}
