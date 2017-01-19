#ifndef MOTOMAN_RVIZ_PLUGINS_EUCLIDEAN_LINK_TRAJECTORY_H_
#define MOTOMAN_RVIZ_PLUGINS_EUCLIDEAN_LINK_TRAJECTORY_H_

#ifndef Q_MOC_RUN
#include <rviz/message_filter_display.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/ogre_helpers/billboard_line.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/message_filter_display.h>
#include <motoman_viz_msgs/EuclideanLinkTrajectory.h>
#endif

namespace motoman_rviz_plugins
{

  class EuclideanLinkTrajectory: public rviz::MessageFilterDisplay<motoman_viz_msgs::EuclideanLinkTrajectory>
  {
    Q_OBJECT
  public:
    EuclideanLinkTrajectory();
    virtual ~EuclideanLinkTrajectory();
  protected:
    void onInitialize();
	virtual void reset();

	rviz::StringProperty* link_name_property_;
	rviz::FloatProperty* alpha_property_;
    rviz::ColorProperty* color_property_;
    rviz::FloatProperty* line_width_property_;
    rviz::BillboardLine* line_;
	std::string link_name_;
	float alpha_;
    QColor color_;
    float line_width_;
	protected Q_SLOTS:
	  void updateLinkName();
	  void updateAlpha();
	  void updateColor();
	  void updateLineWidth();
  private:
	  void processMessage(
						  const motoman_viz_msgs::EuclideanLinkTrajectory::ConstPtr& msg);
  };
}

#endif
