#include <gst/gst.h>
#include <glib.h>
#include <stdio.h>
#include <cuda_runtime_api.h>
#include "gstnvdsmeta.h"

#define MAX_DISPLAY_LEN 64

#define PGIE_CLASS_ID_VEHICLE 0
#define PGIE_CLASS_ID_PERSON 2

/* The muxer output resolution must be set if the input streams will be of
 * different resolution. The muxer will scale all the input frames to this
 * resolution. */
#define MUXER_OUTPUT_WIDTH 1280
#define MUXER_OUTPUT_HEIGHT 960

/* Muxer batch formation timeout, for e.g. 40 millisec. Should ideally be set
 * based on the fastest source's framerate. */
#define MUXER_BATCH_TIMEOUT_USEC 40000

gint frame_number = 0;
gchar pgie_classes_str[4][32] = { "Vehicle", "TwoWheeler", "Person",
  "Roadsign"
};

/* We create functions to make new elements and caps, link them together */
GstElement  *makeElementAndLink(const gchar *factoryname, const gchar *name, \
                GstBin *bin, GstElement *src)
{
  GstElement  *element;
  gboolean    flag;
  element = gst_element_factory_make(factoryname, name);
  flag = gst_bin_add(bin, element);
  g_assert(flag == TRUE);
  if (src != NULL) {
      flag = gst_element_link(src, element);
      g_assert(flag == TRUE);
  }
  return (element);
}

GstElement  *makeCapsAndLink(const gchar *capsStr, const gchar *name, \
                GstBin *bin, GstElement *src)
{
  GstElement  *element;
  GstCaps     *caps;
  caps = gst_caps_from_string(capsStr);
  element = makeElementAndLink("capsfilter", name, bin, src);
  g_object_set(G_OBJECT(element), "caps", caps, NULL);
  gst_caps_unref(caps);
  return (element);
}

/* osd_sink_pad_buffer_probe  will extract metadata received on OSD sink pad
 * and update params for drawing rectangle, object information etc. */

static GstPadProbeReturn osd_sink_pad_buffer_probe (GstPad * pad, GstPadProbeInfo * info,
    gpointer u_data)
{
  GstBuffer *buf = (GstBuffer *) info->data;
  guint num_rects = 0; 
  NvDsObjectMeta *obj_meta = NULL;
  guint vehicle_count = 0;
  guint person_count = 0;
  NvDsMetaList * l_frame = NULL;
  NvDsMetaList * l_obj = NULL;
  NvDsDisplayMeta *display_meta = NULL;

  NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta (buf);

  for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
    l_frame = l_frame->next) {
      NvDsFrameMeta *frame_meta = (NvDsFrameMeta *) (l_frame->data);
      int offset = 0;
      for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
              l_obj = l_obj->next) {
          obj_meta = (NvDsObjectMeta *) (l_obj->data);
          if (obj_meta->class_id == PGIE_CLASS_ID_VEHICLE) {
              vehicle_count++;
              num_rects++;
          }
          if (obj_meta->class_id == PGIE_CLASS_ID_PERSON) {
              person_count++;
              num_rects++;
          }
      }
      display_meta = nvds_acquire_display_meta_from_pool(batch_meta);
      NvOSD_TextParams *txt_params  = &display_meta->text_params[0];
      display_meta->num_labels = 1;
      txt_params->display_text = g_malloc0 (MAX_DISPLAY_LEN);
      offset = snprintf(txt_params->display_text, MAX_DISPLAY_LEN, "Person = %d ", person_count);
      offset = snprintf(txt_params->display_text + offset , MAX_DISPLAY_LEN, "Vehicle = %d ", vehicle_count);

      /* Now set the offsets where the string should appear */
      txt_params->x_offset = 10;
      txt_params->y_offset = 12;

      /* Font , font-color and font-size */
      txt_params->font_params.font_name = "Serif";
      txt_params->font_params.font_size = 10;
      txt_params->font_params.font_color.red = 1.0;
      txt_params->font_params.font_color.green = 1.0;
      txt_params->font_params.font_color.blue = 1.0;
      txt_params->font_params.font_color.alpha = 1.0;

      /* Text background color */
      txt_params->set_bg_clr = 1;
      txt_params->text_bg_clr.red = 0.0;
      txt_params->text_bg_clr.green = 0.0;
      txt_params->text_bg_clr.blue = 0.0;
      txt_params->text_bg_clr.alpha = 1.0;

      nvds_add_display_meta_to_frame(frame_meta, display_meta);
    }

    g_print ("Frame Number = %d Number of objects = %d "
            "Vehicle Count = %d Person Count = %d\n",
            frame_number, num_rects, vehicle_count, person_count);
    frame_number++;
    return GST_PAD_PROBE_OK;
}

static gboolean bus_call (GstBus * bus, GstMessage * msg, gpointer data)
{
  GMainLoop *loop = (GMainLoop *) data;
  switch (GST_MESSAGE_TYPE (msg)) {
    case GST_MESSAGE_EOS:
      g_print ("End of stream\n");
      g_main_loop_quit (loop);
      break;
    case GST_MESSAGE_ERROR:{
      gchar *debug;
      GError *error;
      gst_message_parse_error (msg, &error, &debug);
      g_printerr ("ERROR from element %s: %s\n",
          GST_OBJECT_NAME (msg->src), error->message);
      if (debug)
        g_printerr ("Error details: %s\n", debug);
      g_free (debug);
      g_error_free (error);
      g_main_loop_quit (loop);
      break;
    }
    default:
      break;
  }
  return TRUE;
}

gboolean constructPlipeline(const gchar *device, GstBin *bin)
{
  GstElement  *element = NULL;
  GstCaps     *caps = NULL;
  GstPad      *sinkPad = NULL;
  GstPad      *srcPad = NULL;

  element = makeElementAndLink("v4l2src", NULL, bin, element);
  g_object_set(G_OBJECT(element), "device", "/dev/video1", NULL);

  element = makeCapsAndLink( \
      "video/x-raw, width=1280, height=720, format=YUY2", NULL, bin, element);

  element = makeElementAndLink("videoconvert", NULL, bin, element);

  element = makeCapsAndLink("video/x-raw, format=NV12", NULL, bin, element);

  element = makeElementAndLink("nvvideoconvert", NULL, bin, element);

  element = makeCapsAndLink("video/x-raw(memory:NVMM), format=NV12", \
      NULL, bin, element);

  srcPad = gst_element_get_static_pad(element, "src");
  if (srcPad == NULL) {
      g_print("src pad error\n");
      return (FALSE);
  }

  element = makeElementAndLink("nvstreammux", NULL, bin, NULL);   
  g_object_set(G_OBJECT(element), \
      "width", MUXER_OUTPUT_WIDTH, \
      "height", MUXER_OUTPUT_HEIGHT, \
      "batch-size", 1, \
      "batched-push-timeout", MUXER_BATCH_TIMEOUT_USEC, \
      "live-source", TRUE, \
      NULL);

  sinkPad = gst_element_get_request_pad(element, "sink_0");
  if (sinkPad == NULL) {
      g_print("sink pad error\n");
      return (FALSE);
  }

  if (gst_pad_link(srcPad, sinkPad) != GST_PAD_LINK_OK) {
      g_print("pad link error\n");
      return (FALSE);
  }
  gst_object_unref(GST_OBJECT(sinkPad));
  gst_object_unref(GST_OBJECT(srcPad));

  element = makeElementAndLink("nvinfer", NULL, bin, element);
  /* Set all the necessary properties of the nvinfer element,
  * the necessary ones are : */
  g_object_set(G_OBJECT(element), \
      "config-file-path", "dstest1_pgie_config.txt", NULL);
      
  element = makeCapsAndLink("video/x-raw(memory:NVMM), format=NV12", \
      NULL, bin, element);

  element = makeElementAndLink("nvvideoconvert", NULL, bin, element);

  element = makeCapsAndLink("video/x-raw(memory:NVMM), format=RGBA", \
      NULL, bin, element);

  element = makeElementAndLink("nvdsosd", NULL, bin, element);

  /* Lets add probe to get informed of the meta data generated, we add probe to
  * the sink pad of the osd element, since by that time, the buffer would have
  * had got all the metadata. */
  sinkPad = gst_element_get_static_pad(element, "sink");
  if (!sinkPad) {
      g_print ("Unable to get sink pad\n");
      return (FALSE);
  }
  gst_pad_add_probe(sinkPad, GST_PAD_PROBE_TYPE_BUFFER, \
      osd_sink_pad_buffer_probe, NULL, NULL);
  gst_object_unref(GST_OBJECT(sinkPad));

  #ifdef PLATFORM_TEGRA
    element = makeElementAndLink("nvegltransform", NULL, bin, element);
  #endif
  element = makeElementAndLink("nveglglessink", NULL, bin, element);

  return (TRUE);
}

int main (int argc, char *argv[])
{
  GMainLoop *loop = NULL;
  GstElement *pipeline = NULL;
  GstBus *bus = NULL;
  guint bus_watch_id;
  gchar *device = NULL;
  const gchar *device2 = "/dev/video1";
  GOptionEntry entries[] = {
      {"device", 'd', G_OPTION_FLAG_NONE, G_OPTION_ARG_FILENAME, &device, \
          "Camera device", NULL}, \
      {NULL}
  };

  /* Parse application specific arguments */
  GOptionContext  *ctx = NULL;
  GError      *err = NULL;
  GIOChannel  *io = NULL;
  guint       tag = 0;  
  ctx = g_option_context_new("[APPL_OPTION...]");
  g_option_context_add_main_entries(ctx, entries, NULL);
  g_option_context_add_group(ctx, gst_init_get_option_group());
  if (!g_option_context_parse(ctx, &argc, &argv, &err)) {
      g_print("Failed to initialize: %s\n", err->message);
      g_error_free(err);
      return (-1);
  }
//   if (device == NULL) {
//       device2 = "/dev/video0";
//   }
//   else {
//       device2 = device;
//   }

  /* Standard GStreamer initialization */
  gst_init (&argc, &argv);
  loop = g_main_loop_new (NULL, FALSE);

  /* Create gstreamer elements */
  /* Create Pipeline element that will form a connection of other elements */
  pipeline = gst_pipeline_new ("dstest1-pipeline");

  /* Construct the pipeline */
  if (!constructPlipeline(device2, GST_BIN(pipeline))) {
      return (-1);
  }
  g_free(device);

  /* we add a message handler */
  bus = gst_pipeline_get_bus (GST_PIPELINE (pipeline));
  bus_watch_id = gst_bus_add_watch (bus, bus_call, loop);
  gst_object_unref (bus);

  /* Set the pipeline to "playing" state */
  g_print ("Now playing: %s\n", argv[1]);
  gst_element_set_state (pipeline, GST_STATE_PLAYING);

  /* Wait till pipeline encounters an error or EOS */
  g_print ("Running...\n");
  g_main_loop_run (loop);

  /* Out of the main loop, clean up nicely */
  g_print ("Returned, stopping playback\n");
  gst_element_set_state (pipeline, GST_STATE_NULL);
  g_print ("Deleting pipeline\n");
  gst_object_unref (GST_OBJECT (pipeline));
  g_source_remove (bus_watch_id);
  g_main_loop_unref (loop);
  return 0;
}
