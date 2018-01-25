#include "euclidean_link_trajectory.h"

namespace motoman_rviz_plugins
{
  EuclideanLinkTrajectory::EuclideanLinkTrajectory()
  {
	link_name_property_ = new rviz::StringProperty("link name", "", "",
												   this, SLOT(updateLinkName()));
	alpha_property_ = new rviz::FloatProperty("alpha", 1.0,
											  "alpha channel value",
											  this, SLOT(updateAlpha()));
	color_property_ = new rviz::ColorProperty("color", QColor(25, 255, 240),
                                              "color of trajectory",
											  this, SLOT(updateColor()));
    line_width_property_ = new rviz::FloatProperty("line_width", 0.01,
                                                   "line width",
                                                   this, SLOT(updateLineWidth()));
	alpha_property_->setMin(0.0);
    line_width_property_->setMin(0.0);
  }

  EuclideanLinkTrajectory::~EuclideanLinkTrajectory()
  {
	delete link_name_property_;
	delete alpha_property_;
	delete color_property_;
    delete line_width_property_;
    delete line_;
  }

  void EuclideanLinkTrajectory::onInitialize()
  {
	MFDClass::onInitialize();
    line_ = new rviz::BillboardLine(context_->getSceneManager(), scene_node_);
	updateLinkName();
	updateAlpha();
	updateColor();
	updateLineWidth();
  }

  void EuclideanLinkTrajectory::reset()
  {
	MFDClass::reset();
	line_->clear();
  }
  
  void EuclideanLinkTrajectory::updateLinkName()
  {
  	link_name_ = link_name_property_->getStdString();
  }

  void EuclideanLinkTrajectory::updateAlpha()
  {
	alpha_ = alpha_property_->getFloat();
	line_->setColor(color_.red() * 255.0, color_.green() * 255.0, color_.blue() * 255.0, alpha_);
  }

  void EuclideanLinkTrajectory::updateColor()
  {
    color_ = color_property_->getColor();
	line_->setColor(color_.red() * 255.0, color_.green() * 255.0, color_.blue() * 255.0, alpha_);
  }

  void EuclideanLinkTrajectory::updateLineWidth()
  {
    line_width_ = line_width_property_->getFloat();
	line_->setLineWidth(line_width_);
  }
  
  void EuclideanLinkTrajectory::processMessage(
											   const motoman_viz_msgs::EuclideanLinkTrajectory::ConstPtr& msg){
	
	std::map<std::string, int> link_map;
	for(int i=0; i<msg->link_names.size(); i++)
	  link_map.insert(std::pair<std::string, int>(msg->link_names[i], i));
	
	line_->clear();
	line_->setNumLines(1);
	line_->setMaxPointsPerLine(msg->points.size());
	line_->setLineWidth(line_width_);
	line_->setColor(color_.red() * 255.0, color_.green() * 255.0, color_.blue() * 255.0, alpha_);
	std::vector<Ogre::Vector3> p(msg->points.size());
	for(size_t i=0; i<p.size(); i++){
	  p[i][0] = msg->points[i].pose[link_map[link_name_]].position.x;
	  p[i][1] = msg->points[i].pose[link_map[link_name_]].position.y;
	  p[i][2] = msg->points[i].pose[link_map[link_name_]].position.z;
	  line_->addPoint(p[i]);
	}
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( motoman_rviz_plugins::EuclideanLinkTrajectory, rviz::Display )
