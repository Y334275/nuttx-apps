/****************************************************************************
 * apps/testing/mavlink_otg/mavlink_otg_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Minimal MAVLink v2 target for ESP32-S3 OTG TX/RX and reconnect testing.
 *
 * TX: HEARTBEAT at 10 Hz, with an incrementing custom_mode counter.
 * RX: PARAM_REQUEST_LIST -> PARAM_VALUE, PARAM_SET -> PARAM_VALUE ACK.
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/boardctl.h>
#include <sys/types.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MAV_MAGIC_V2       0xfd
#define MAV_SYSID          1
#define MAV_COMPID         1

#define MSG_HEARTBEAT      0
#define MSG_PARAM_REQ_LIST 21
#define MSG_PARAM_VALUE    22
#define MSG_PARAM_SET      23

#define CRC_HEARTBEAT      50
#define CRC_PARAM_REQ_LIST 159
#define CRC_PARAM_VALUE    220
#define CRC_PARAM_SET      168

#define PARAM_COUNT        16
#define PARAM_ID_LEN       16
#define PARAM_VALUE_LEN    25
#define HEARTBEAT_LEN      9
#define PARAM_REQ_LEN      2
#define PARAM_SET_LEN      23

#define RX_SIZE            512
#define HEARTBEAT_MS       100

static uint8_t g_rx[RX_SIZE];
static size_t g_rxlen;
static uint8_t g_seq;
static uint32_t g_tx_counter;


static const char *g_param_names[PARAM_COUNT] =
{
  "TEST_P0",  "TEST_P1",  "TEST_P2",  "TEST_P3",
  "TEST_P4",  "TEST_P5",  "TEST_P6",  "TEST_P7",
  "TEST_P8",  "TEST_P9",  "TEST_P10", "TEST_P11",
  "TEST_P12", "TEST_P13", "TEST_P14", "TEST_P15"
};

static float g_param_values[PARAM_COUNT];

static uint16_t crc_accumulate(uint16_t crc, uint8_t data)
{
  uint8_t tmp = data ^ (uint8_t)(crc & 0xff);
  tmp = (uint8_t)(tmp ^ (tmp << 4));
  return (uint16_t)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

static uint16_t mavlink_crc(const uint8_t *buf, size_t len, uint8_t extra)
{
  uint16_t crc = 0xffff;

  for (size_t i = 0; i < len; i++)
    {
      crc = crc_accumulate(crc, buf[i]);
    }

  return crc_accumulate(crc, extra);
}

static int write_all(int fd, const uint8_t *buf, size_t len)
{
  size_t done = 0;
  int retries = 0;

  while (done < len)
    {
      ssize_t n = write(fd, buf + done, len - done);
      if (n < 0)
        {
          if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
              struct pollfd pfd;

              if (++retries > 10)
                {
                  return -1;
                }

              pfd.fd = fd;
              pfd.events = POLLOUT;
              (void)poll(&pfd, 1, 10);
              continue;
            }

          return -1;
        }

      if (n == 0)
        {
          if (++retries > 10)
            {
              return -1;
            }

          usleep(1000);
          continue;
        }

      retries = 0;
      done += (size_t)n;
    }

  return 0;
}

static int send_frame(int fd, uint8_t msgid, const uint8_t *payload,
                      uint8_t payload_len, uint8_t crc_extra)
{
  uint8_t frame[10 + 255 + 2];
  size_t len = 0;
  uint16_t crc;

  frame[len++] = MAV_MAGIC_V2;
  frame[len++] = payload_len;
  frame[len++] = 0;                 /* incompat_flags */
  frame[len++] = 0;                 /* compat_flags */
  frame[len++] = g_seq++;
  frame[len++] = MAV_SYSID;
  frame[len++] = MAV_COMPID;
  frame[len++] = msgid;
  frame[len++] = 0;
  frame[len++] = 0;

  if (payload_len > 0)
    {
      memcpy(&frame[len], payload, payload_len);
      len += payload_len;
    }

  crc = mavlink_crc(&frame[1], len - 1, crc_extra);
  frame[len++] = (uint8_t)(crc & 0xff);
  frame[len++] = (uint8_t)(crc >> 8);

  return write_all(fd, frame, len);
}

static int send_heartbeat(int fd)
{
  uint8_t payload[HEARTBEAT_LEN];
  uint32_t counter = g_tx_counter++;

  memset(payload, 0, sizeof(payload));
  payload[0] = (uint8_t)(counter & 0xff);
  payload[1] = (uint8_t)((counter >> 8) & 0xff);
  payload[2] = (uint8_t)((counter >> 16) & 0xff);
  payload[3] = (uint8_t)((counter >> 24) & 0xff);
  payload[4] = 2;                   /* MAV_TYPE_QUADROTOR */
  payload[5] = 12;                  /* MAV_AUTOPILOT_PX4 */
  payload[6] = 0;
  payload[7] = 4;                   /* MAV_STATE_ACTIVE */
  payload[8] = 3;                   /* MAVLink version */

  return send_frame(fd, MSG_HEARTBEAT, payload, sizeof(payload),
                    CRC_HEARTBEAT);
}

static int send_param_value(int fd, uint16_t index)
{
  uint8_t payload[PARAM_VALUE_LEN];
  union
  {
    float f;
    uint8_t b[4];
  } value;

  memset(payload, 0, sizeof(payload));
  strncpy((char *)payload, g_param_names[index], PARAM_ID_LEN);
  value.f = g_param_values[index];
  memcpy(&payload[PARAM_ID_LEN], value.b, 4);
  payload[PARAM_ID_LEN + 4] = 9;    /* MAV_PARAM_TYPE_REAL32 */
  payload[PARAM_ID_LEN + 5] = (uint8_t)(PARAM_COUNT & 0xff);
  payload[PARAM_ID_LEN + 6] = (uint8_t)(PARAM_COUNT >> 8);
  payload[PARAM_ID_LEN + 7] = (uint8_t)(index & 0xff);
  payload[PARAM_ID_LEN + 8] = (uint8_t)(index >> 8);

  return send_frame(fd, MSG_PARAM_VALUE, payload, sizeof(payload),
                    CRC_PARAM_VALUE);
}

static void send_all_params(int fd)
{
  for (uint16_t i = 0; i < PARAM_COUNT; i++)
    {
      (void)send_param_value(fd, i);
    }
}

static void handle_frame(int fd, const uint8_t *frame, size_t len)
{
  uint8_t payload_len = frame[1];
  uint8_t msgid = frame[7];
  const uint8_t *payload = &frame[10];
  uint8_t crc_extra;

  if (len != (size_t)(10 + payload_len + 2))
    {
      return;
    }

  switch (msgid)
    {
      case MSG_HEARTBEAT:      crc_extra = CRC_HEARTBEAT; break;
      case MSG_PARAM_REQ_LIST: crc_extra = CRC_PARAM_REQ_LIST; break;
      case MSG_PARAM_VALUE:    crc_extra = CRC_PARAM_VALUE; break;
      case MSG_PARAM_SET:      crc_extra = CRC_PARAM_SET; break;
      default: return;
    }

  uint16_t crc = mavlink_crc(&frame[1], 9 + payload_len, crc_extra);
  if (frame[10 + payload_len] != (uint8_t)(crc & 0xff) ||
      frame[11 + payload_len] != (uint8_t)(crc >> 8))
    {
      return;
    }

  if (msgid == MSG_PARAM_REQ_LIST && payload_len >= PARAM_REQ_LEN)
    {
      send_all_params(fd);
    }
  else if (msgid == MSG_PARAM_SET && payload_len >= PARAM_SET_LEN)
    {
      char id[PARAM_ID_LEN + 1];
      union
      {
        float f;
        uint8_t b[4];
      } value;

      memcpy(id, payload, PARAM_ID_LEN);
      id[PARAM_ID_LEN] = '\0';
      memcpy(value.b, &payload[PARAM_ID_LEN], 4);

      for (uint16_t i = 0; i < PARAM_COUNT; i++)
        {
          if (strncmp(id, g_param_names[i], PARAM_ID_LEN) == 0)
            {
              g_param_values[i] = value.f;
              (void)send_param_value(fd, i);
              break;
            }
        }
    }
}

static void parse_rx(int fd)
{
  size_t consumed = 0;

  while (consumed < g_rxlen)
    {
      if (g_rx[consumed] != MAV_MAGIC_V2)
        {
          consumed++;
          continue;
        }

      if (g_rxlen - consumed < 10)
        {
          break;
        }

      uint8_t payload_len = g_rx[consumed + 1];
      size_t frame_len = 10 + payload_len + 2;

      if (g_rxlen - consumed < frame_len)
        {
          break;
        }

      handle_frame(fd, &g_rx[consumed], frame_len);
      consumed += frame_len;
    }

  if (consumed > 0)
    {
      memmove(g_rx, &g_rx[consumed], g_rxlen - consumed);
      g_rxlen -= consumed;
    }
}

static int usbdev_connect(void)
{
  struct boardioc_usbdev_ctrl_s ctrl;
  void *handle = NULL;
  int ret;

  memset(&ctrl, 0, sizeof(ctrl));
  ctrl.usbdev = BOARDIOC_USBDEV_CDCACM;
  ctrl.action = BOARDIOC_USBDEV_CONNECT;
  ctrl.instance = 0;
  ctrl.handle = &handle;

  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  return ret < 0 ? -errno : 0;
}

static int usbdev_disconnect(void)
{
  struct boardioc_usbdev_ctrl_s ctrl;
  int ret;

  memset(&ctrl, 0, sizeof(ctrl));
  ctrl.usbdev = BOARDIOC_USBDEV_CDCACM;
  ctrl.action = BOARDIOC_USBDEV_DISCONNECT;
  ctrl.instance = 0;
  ctrl.handle = NULL;

  ret = boardctl(BOARDIOC_USBDEV_CONTROL, (uintptr_t)&ctrl);
  return ret < 0 ? -errno : 0;
}

static int open_cdc(void)
{
  int fd;

  for (;;)
    {
      fd = open(CONFIG_TESTING_MAVLINK_OTG_DEVPATH, O_RDWR | O_NONBLOCK);
      if (fd >= 0)
        {
          return fd;
        }

      usleep(100 * 1000);
    }
}

int main(int argc, FAR char *argv[])
{
  uint8_t buf[128];
  struct timespec last;
  struct timespec last_stress;
  int fd;

  for (uint16_t i = 0; i < PARAM_COUNT; i++)
    {
      g_param_values[i] = (float)i;
    }

#ifdef CONFIG_TESTING_MAVLINK_OTG_NO_USB

  for (;;)
    {
      sleep(1);
    }
#endif

  (void)usbdev_connect();
  fd = open_cdc();
  clock_gettime(CLOCK_MONOTONIC, &last);
  clock_gettime(CLOCK_MONOTONIC, &last_stress);
  (void)send_heartbeat(fd);

#ifdef CONFIG_TESTING_MAVLINK_OTG_REBIND
  /* Exercise the class reset/reconfigure path without a host USB reset. */

  sleep(5);
  close(fd);
  (void)usbdev_disconnect();
  usleep(100 * 1000);
  (void)usbdev_connect();
  fd = open_cdc();
  clock_gettime(CLOCK_MONOTONIC, &last);
  (void)send_heartbeat(fd);
#endif

  for (;;)
    {
      struct pollfd pfd;
      pfd.fd = fd;
      pfd.events = POLLIN;

      int pr = poll(&pfd, 1, HEARTBEAT_MS);

      if (pr > 0 && (pfd.revents & POLLIN) != 0)
        {
          ssize_t n = read(fd, buf, sizeof(buf));
          if (n > 0)
            {
              if (g_rxlen + (size_t)n > sizeof(g_rx))
                {
                  g_rxlen = 0;
                }

              memcpy(&g_rx[g_rxlen], buf, (size_t)n);
              g_rxlen += (size_t)n;
              parse_rx(fd);
            }
        }

      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);

#ifdef CONFIG_TESTING_MAVLINK_OTG_STRESS
      {
        long elapsed_ms;

        elapsed_ms = (now.tv_sec - last_stress.tv_sec) * 1000 +
                     (now.tv_nsec - last_stress.tv_nsec) / 1000000;

        if (elapsed_ms >= 10)
          {
            last_stress = now;
            send_all_params(fd);
          }
      }
#endif

      if ((now.tv_sec - last.tv_sec) >= 0 &&
          ((now.tv_sec - last.tv_sec) * 1000 +
           (now.tv_nsec - last.tv_nsec) / 1000000) >= HEARTBEAT_MS)
        {
          last = now;
          (void)send_heartbeat(fd);
        }
    }

  return 0;
}
