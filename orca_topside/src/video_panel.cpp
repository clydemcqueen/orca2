#include "orca_topside/video_panel.h"

#include "rviz_common/logging.hpp"

// Qt wrappers around some glib and gst headers
#include "QGlib/Connect"
#include "QGst/Bus"
#include "QGst/ElementFactory"
#include "QGst/Init"
#include "QGst/Message"

// Other Qt headers
#include "QVBoxLayout"

namespace orca_topside {

VideoPlayer::VideoPlayer(QWidget *parent) : QGst::Ui::VideoWidget(parent)
{
  RVIZ_COMMON_LOG_DEBUG("Constructing VideoPlayer");

  // Init GStreamer
  QGst::init();

  try
  {
    // Build a GStreamer pipeline
    RVIZ_COMMON_LOG_DEBUG("Creating gstreamer pipeline");
    pipeline_ = QGst::Pipeline::create();
    QGst::ElementPtr bin = QGst::Bin::fromDescription("udpsrc uri=udp://0.0.0.0:5600 "
      "caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264\" "
      "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink");
    pipeline_->add(bin);

    RVIZ_COMMON_LOG_DEBUG("Watching the pipeline bus for error messages");
    pipeline_->bus()->addSignalWatch();
    QGlib::connect(pipeline_->bus(), "message", this, &VideoPlayer::onBusMessage);

    RVIZ_COMMON_LOG_DEBUG("Watching the pipeline bus for sync-messages");
    watchPipeline(pipeline_); // Calls this->setVideoSink when a video sink appears

    RVIZ_COMMON_LOG_DEBUG("Setting state to playing");
    if (pipeline_->setState(QGst::StatePlaying) == QGst::StateChangeReturn::StateChangeFailure)
    {
      RVIZ_COMMON_LOG_ERROR("Can't set state to playing");
    }
  }
  catch (const QGlib::Error& error)
  {
    RVIZ_COMMON_LOG_ERROR_STREAM("Can't create video pipeline, error: " << error.message().toStdString());
  }
}

VideoPlayer::~VideoPlayer()
{
  RVIZ_COMMON_LOG_DEBUG("Destroying VideoPlayer");
  pipeline_->setState(QGst::StateNull);
  pipeline_->bus()->removeSignalWatch();
  stopPipelineWatch();
}

void VideoPlayer::onBusMessage(const QGst::MessagePtr& message)
{
  switch (message->type()) {
    case QGst::MessageEos:
      // End of stream -- shouldn't happen
      RVIZ_COMMON_LOG_ERROR("Video stream stopped");
      break;
    case QGst::MessageError:
      RVIZ_COMMON_LOG_ERROR_STREAM("QGst error: " << message.staticCast<QGst::ErrorMessage>()->error().message().toStdString());
      break;
    case QGst::MessageStateChanged:
      // The element in message->source() has changed state
      if (message->source() == pipeline_)
      {
        RVIZ_COMMON_LOG_INFO_STREAM("QGst state change: " << message.staticCast<QGst::StateChangedMessage>()->newState());
      }
      break;
    default:
      // Ignore
      break;
  }
}

VideoPanel::VideoPanel(QWidget *parent) : rviz_common::Panel(parent)
{
  RVIZ_COMMON_LOG_DEBUG("Constructing VideoPanel");

  QVBoxLayout* layout = new QVBoxLayout;
  player_ = new VideoPlayer(this);
  layout->addWidget(player_);
  setLayout(layout);
}

VideoPanel::~VideoPanel()
{
  RVIZ_COMMON_LOG_DEBUG("Destroying VideoPanel");

  delete player_;
}

} // namespace orca_topside

// Tell pluginlib about this class
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(orca_topside::VideoPanel, rviz_common::Panel)