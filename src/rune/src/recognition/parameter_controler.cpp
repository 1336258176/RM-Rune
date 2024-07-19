#include "recognition/recognition_node.hpp"

namespace rune
{
void RecognitionNode::declareParams()
{
  try
  {
    const auto enemy_color_str = this->declare_parameter("lightbar_color", "Blue");
    rune_color_                = enemy_color_str == "Blue" ? LightColor::RED : LightColor::BLUE;

    /* debug params */
    debug_ = this->declare_parameter<bool>("debug", debug_);
    debug_params_->raw_img_ =
      this->declare_parameter<bool>("debug_raw_img", debug_params_->raw_img_);
    debug_params_->bin_img_ =
      this->declare_parameter<bool>("debug_bin_img", debug_params_->bin_img_);
    debug_params_->con_img_ =
      this->declare_parameter<bool>("debug_con_img", debug_params_->con_img_);
    debug_params_->rec_img_ =
      this->declare_parameter<bool>("debug_rec_img", debug_params_->rec_img_);
    debug_params_->show_err_contours_ = 
      this->declare_parameter<bool>("show_err_contours", debug_params_->show_err_contours_);
    debug_params_->record_rec_img_ =
      this->declare_parameter<bool>("record_rec_img", debug_params_->record_rec_img_);
    debug_params_->show_pnp =
      this->declare_parameter<bool>("debug_show_pnp", debug_params_->show_pnp);

    /* rune params */
    rune_params_->BinaryThres_Blue =
      this->declare_parameter<double>("BinaryThres_Blue", rune_params_->BinaryThres_Blue);
    rune_params_->BinartThres_Red =
      this->declare_parameter<double>("BinaryThres_Red", rune_params_->BinartThres_Red);
    rune_params_->BinaryThres =
      this->declare_parameter<double>("BinaryThres", rune_params_->BinaryThres);
    rune_params_->MinWaterfallLightCount = 
      this->declare_parameter<int>("MinWaterfallLightCount", rune_params_->MinWaterfallLightCount);
    rune_params_->RuneRadius =
      this->declare_parameter<double>("RuneRadius", rune_params_->RuneRadius);
    rune_params_->RHeight  = this->declare_parameter<double>("RHeight", rune_params_->RHeight);
    rune_params_->WaterfallLightMaxDistanceDiff = this->declare_parameter<double>("WaterfallLightMaxDistanceDiff", 
                                                                                  rune_params_->WaterfallLightMaxDistanceDiff);
    rune_params_->WaterfallLightMinArea = this->declare_parameter<double>("WaterfallLightMinArea", rune_params_->WaterfallLightMinArea);
    rune_params_->WaterfallLightMaxArea = this->declare_parameter<double>("WaterfallLightMaxArea", rune_params_->WaterfallLightMaxArea);
    rune_params_->WaterfallLightMinROF  = this->declare_parameter<double>("WaterfallLightMinROF", rune_params_->WaterfallLightMinROF);
    rune_params_->WaterfallLightMaxROF  = this->declare_parameter<double>("WaterfallLightMaxROF", rune_params_->WaterfallLightMaxROF);
    rune_params_->WaterfallLightMinAspectRatio =
      this->declare_parameter<double>("WaterfallLightMinAspectRatio", rune_params_->WaterfallLightMinAspectRatio);
    rune_params_->WaterfallLightMaxAspectRatio =
      this->declare_parameter<double>("WaterfallLightMaxAspectRatio", rune_params_->WaterfallLightMaxAspectRatio);
    rune_params_->MoonFrameMinArea = this->declare_parameter<double>("MoonFrameMinArea", rune_params_->MoonFrameMinArea);
    rune_params_->MoonFrameMaxArea = this->declare_parameter<double>("MoonFrameMaxArea", rune_params_->MoonFrameMaxArea);
    rune_params_->MoonFrameMinAspectRatio =
      this->declare_parameter<double>("MoonFrameMinAspectRatio", rune_params_->MoonFrameMinAspectRatio);
    rune_params_->MoonFrameMaxAspectRatio =
      this->declare_parameter<double>("MoonFrameMaxAspectRatio", rune_params_->MoonFrameMaxAspectRatio);
    rune_params_->MoonFrameMinROF   = this->declare_parameter<double>("MoonFrameMinROF", rune_params_->MoonFrameMinROF);
    rune_params_->MoonFrameMaxROF   = this->declare_parameter<double>("MoonFrameMaxROF", rune_params_->MoonFrameMaxROF);
    rune_params_->InsideMoonWidth    = this->declare_parameter<double>("InsideMoonWidth", rune_params_->InsideMoonWidth);
    rune_params_->InsideMoonLength   = this->declare_parameter<double>("InsideMoonLength", rune_params_->InsideMoonLength);
    rune_params_->OutsideMoonWidth = this->declare_parameter<double>("OutsideMoonWidth", rune_params_->OutsideMoonWidth);
    rune_params_->OutsideMoonHeight =
      this->declare_parameter<double>("OutsideMoonHeight", rune_params_->OutsideMoonHeight);
    rune_params_->FanbladeCenter2InsideMoontop =
      this->declare_parameter<double>("FanbladeCenter2InsideMoontop", rune_params_->FanbladeCenter2InsideMoontop);
    rune_params_->FanbladeCenter2OutsideMoontop = this->declare_parameter<double>(
      "FanbladeCenter2OutsideMoontop", rune_params_->FanbladeCenter2OutsideMoontop);
    rune_params_->FanBladeCenter_Outside_Inside_1 = this->declare_parameter<double>(
      "FanBladeCenter_Outside_Inside_1", rune_params_->FanBladeCenter_Outside_Inside_1);
    rune_params_->FanBladeCenter_Outside_Inside_2 = this->declare_parameter<double>(
      "FanBladeCenter_Outside_Inside_2", rune_params_->FanBladeCenter_Outside_Inside_2);

    /* ros topic name */
    ros_topic_name_->SubImgTopicName =
      this->declare_parameter("SubImgTopicName", ros_topic_name_->SubImgTopicName);
    ros_topic_name_->SubCameraInfoName =
      this->declare_parameter("SubCameraInfoName", ros_topic_name_->SubCameraInfoName);
    ros_topic_name_->PubFanBladeTopicName =
      this->declare_parameter("PubFanBladeTopicName", ros_topic_name_->PubFanBladeTopicName);
    ros_topic_name_->PubImgTopicName_debug_raw = this->declare_parameter(
      "PubImgTopicName_debug_raw", ros_topic_name_->PubImgTopicName_debug_raw);
    ros_topic_name_->PubImgTopicName_debug_bin = this->declare_parameter(
      "PubImgTopicName_debug_bin", ros_topic_name_->PubImgTopicName_debug_bin);
    ros_topic_name_->PubImgTopicName_debug_con = this->declare_parameter(
      "PubImgTopicName_debug_con", ros_topic_name_->PubImgTopicName_debug_con);
    ros_topic_name_->PubImgTopicName_debug_rec = this->declare_parameter(
      "PubImgTopicName_debug_rec", ros_topic_name_->PubImgTopicName_debug_rec);
  } catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Rune-DeclareParams Error: %s", e.what());
  }
}

void RecognitionNode::updateParams()
{
  try
  {
    const auto enemy_color_str = this->get_parameter("lightbar_color").as_string();
    // RCLCPP_INFO(this->get_logger(), "enemy_color_str: %s", enemy_color_str.c_str());
    rune_color_ = enemy_color_str == "Blue" ? LightColor::RED : LightColor::BLUE;

    /* debug params */
    debug_                         = this->get_parameter("debug").as_bool();
    debug_params_->raw_img_        = this->get_parameter("debug_raw_img").as_bool();
    debug_params_->bin_img_        = this->get_parameter("debug_bin_img").as_bool();
    debug_params_->con_img_        = this->get_parameter("debug_con_img").as_bool();
    debug_params_->rec_img_        = this->get_parameter("debug_rec_img").as_bool();
    debug_params_->record_rec_img_ = this->get_parameter("record_rec_img").as_bool();
    debug_params_->show_pnp        = this->get_parameter("debug_show_pnp").as_bool();
    debug_params_->show_err_contours_ = this->get_parameter("show_err_contours").as_bool();

    /* rune params */
    rune_params_->BinaryThres_Blue    = this->get_parameter("BinaryThres_Blue").as_double();
    rune_params_->BinartThres_Red     = this->get_parameter("BinaryThres_Red").as_double();
    rune_params_->BinaryThres         = this->get_parameter("BinaryThres").as_double();
    rune_params_->MinWaterfallLightCount = this->get_parameter("MinWaterfallLightCount").as_int();
    rune_params_->RuneRadius          = this->get_parameter("RuneRadius").as_double();
    rune_params_->RHeight             = this->get_parameter("RHeight").as_double();
    rune_params_->WaterfallLightMaxDistanceDiff = this->get_parameter("WaterfallLightMaxDistanceDiff").as_double();
    rune_params_->WaterfallLightMinArea            = this->get_parameter("WaterfallLightMinArea").as_double();
    rune_params_->WaterfallLightMaxArea            = this->get_parameter("WaterfallLightMaxArea").as_double();
    rune_params_->WaterfallLightMinROF             = this->get_parameter("WaterfallLightMinROF").as_double();
    rune_params_->WaterfallLightMaxROF             = this->get_parameter("WaterfallLightMaxROF").as_double();
    rune_params_->WaterfallLightMinAspectRatio     = this->get_parameter("WaterfallLightMinAspectRatio").as_double();
    rune_params_->WaterfallLightMaxAspectRatio     = this->get_parameter("WaterfallLightMaxAspectRatio").as_double();
    rune_params_->MoonFrameMinArea            = this->get_parameter("MoonFrameMinArea").as_double();
    rune_params_->MoonFrameMaxArea            = this->get_parameter("MoonFrameMaxArea").as_double();
    rune_params_->MoonFrameMinAspectRatio     = this->get_parameter("MoonFrameMinAspectRatio").as_double();
    rune_params_->MoonFrameMaxAspectRatio     = this->get_parameter("MoonFrameMaxAspectRatio").as_double();
    rune_params_->MoonFrameMinROF             = this->get_parameter("MoonFrameMinROF").as_double();
    rune_params_->MoonFrameMaxROF             = this->get_parameter("MoonFrameMaxROF").as_double();
    rune_params_->InsideMoonWidth              = this->get_parameter("InsideMoonWidth").as_double();
    rune_params_->InsideMoonLength             = this->get_parameter("InsideMoonLength").as_double();
    rune_params_->OutsideMoonWidth           = this->get_parameter("OutsideMoonWidth").as_double();
    rune_params_->OutsideMoonHeight          = this->get_parameter("OutsideMoonHeight").as_double();
    rune_params_->FanbladeCenter2InsideMoontop = this->get_parameter("FanbladeCenter2InsideMoontop").as_double();
    rune_params_->FanbladeCenter2OutsideMoontop =
      this->get_parameter("FanbladeCenter2OutsideMoontop").as_double();
    rune_params_->FanBladeCenter_Outside_Inside_1 =
      this->get_parameter("FanBladeCenter_Outside_Inside_1").as_double();
    rune_params_->FanBladeCenter_Outside_Inside_2 =
      this->get_parameter("FanBladeCenter_Outside_Inside_2").as_double();

    /* ros topic name */
    ros_topic_name_->SubImgTopicName      = this->get_parameter("SubImgTopicName").as_string();
    ros_topic_name_->SubCameraInfoName    = this->get_parameter("SubCameraInfoName").as_string();
    ros_topic_name_->PubFanBladeTopicName = this->get_parameter("PubFanBladeTopicName").as_string();
    ros_topic_name_->PubImgTopicName_debug_raw =
      this->get_parameter("PubImgTopicName_debug_raw").as_string();
    ros_topic_name_->PubImgTopicName_debug_bin =
      this->get_parameter("PubImgTopicName_debug_bin").as_string();
    ros_topic_name_->PubImgTopicName_debug_con =
      this->get_parameter("PubImgTopicName_debug_con").as_string();
    ros_topic_name_->PubImgTopicName_debug_rec =
      this->get_parameter("PubImgTopicName_debug_rec").as_string();
  } catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Rune-UpdateParams Error: %s", e.what());
  }
}
}  // namespace rune
