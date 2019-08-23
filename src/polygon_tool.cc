/*
 * polygon_coverage_planning implements algorithms for coverage planning in
 * general polygons with holes. Copyright (C) 2019, Rik Bähnemann, Autonomous
 * Systems Lab, ETH Zürich
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "rviz_polygon_tool/polygon_tool.h"

namespace rviz_polygon_tool {

// Colors.
const Ogre::ColourValue kRed = Ogre::ColourValue(1.f, 0.f, 0.f, 1.0);
const Ogre::ColourValue kGreen = Ogre::ColourValue(0.f, 1.f, 0.f, 1.0);
const Ogre::ColourValue kBlue = Ogre::ColourValue(0.f, 0.f, 1.f, 1.0);
const Ogre::ColourValue kPink = Ogre::ColourValue(1.f, 0.f, 1.f, 1.0);
const Ogre::ColourValue kYellow = Ogre::ColourValue(1.f, 1.f, 0.f, 1.0);
const Ogre::ColourValue kTransparent = Ogre::ColourValue(0.f, 0.f, 0.f, 0.0);

// Status.
const QString kLeftClick = "<b>Left-Click:</b> Insert a new vertex";
const QString kRightClick = "<b>Right-Click:</b> Remove a vertex";
const QString kV = "<b>v:</b> Select next vertex";
const QString kC = "<b>c:</b> Clear all";
const QString kEnter = "<b>Enter:</b> Publish polygon";
//const QString kMouse = "<b>Mouse wheel (+shift/ctrl):</b> Change altitude";
//const QString kAltitude = "Altitude: ";
const QString kSelection = "Current Selection: ";
const QString kStatus = kLeftClick + ", " + kRightClick + ", " + kV + ", " + kC + ", " + kEnter;

// Point scales.
const float kPtScale = 0.5;
const float kDeleteTol = 0.5;

// Altitude changes.
//const double kSmallAltitudeDelta = 0.05;
//const double kNormalAltitudeDelta = 1.0;
//const double kLargeAltitudeDelta = 10.0;
const double kDefaultAltitude = 0.0;

PolygonTool::PolygonTool()
    : Tool(),
      polygon_(geometry_msgs::Polygon()),
      vertex_selection_(polygon_.points.begin()),
      altitude_(kDefaultAltitude),
      polygon_node_(nullptr),
      moving_vertex_node_(nullptr),
      sphere_(nullptr) {
  shortcut_key_ = 'p';
}

PolygonTool::~PolygonTool() {}

void PolygonTool::onInitialize() {
  // OGRE nodes.
  moving_vertex_node_ =
      scene_manager_->getRootSceneNode()->createChildSceneNode();
  sphere_ =
      new rviz::Shape(rviz::Shape::Sphere, scene_manager_, moving_vertex_node_);
  moving_vertex_node_->setVisible(false);

  polygon_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
  polygon_node_->setVisible(true);

  // ROS.
  polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>(
      "polygon", 1, true);
}

void PolygonTool::activate() {
  // Make nodes visible.
  if (moving_vertex_node_) {
    moving_vertex_node_->setVisible(true);
  }
  if (polygon_node_) {
    polygon_node_->setVisible(true);
  }
}

void PolygonTool::deactivate() {
  // Make nodes invisible.
  if (moving_vertex_node_) {
    moving_vertex_node_->setVisible(false);
  }
  if (polygon_node_) {
    polygon_node_->setVisible(false);
  }
}

int PolygonTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
  if (!moving_vertex_node_) {
    return Render;
  }
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);

  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    moving_vertex_node_->setVisible(false);
    moving_vertex_node_->setPosition(intersection);

    if (event.leftUp()) {
      clickLeft(event);
    } else if (event.rightUp()) {
      clickRight(event);
    }/* else if (event.wheel_delta > 0) {
      increaseAltitude(event);
    } else if (event.wheel_delta < 0) {
      decreaseAltitude(event);
    }*/
  } else {
    // Don't show point if not on plane.
    moving_vertex_node_->setVisible(false);
  }

  return Render;
}

void PolygonTool::clickLeft(const rviz::ViewportMouseEvent& event) {
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    createVertex(intersection);
  }

  renderPolygon();
}

void PolygonTool::createVertex(const Ogre::Vector3& position) {
  // Add a vertex to the current polygon.
  geometry_msgs::Point32 new_point;
  new_point.x = position.x;
  new_point.y = position.y;
  vertex_selection_ = polygon_.points.insert(vertex_selection_, new_point);
}

void PolygonTool::clickRight(const rviz::ViewportMouseEvent& event) {
  Ogre::Vector3 intersection;
  Ogre::Plane polygon_plane(Ogre::Vector3::UNIT_Z, 0.0f);
  if (rviz::getPointOnPlaneFromWindowXY(event.viewport, polygon_plane, event.x,
                                        event.y, intersection)) {
    deleteVertex(intersection);
  }

  renderPolygon();
}

void PolygonTool::deleteVertex(const Ogre::Vector3& position) {
  // Select vertex close by.
  for (auto v = polygon_.points.begin(); v != polygon_.points.end(); ++v) {
    Ogre::Vector3 pt((double)(v->x), (double)(v->y), 0.0);
    if ((position - pt).length() < kDeleteTol) {
      vertex_selection_ = v;
      vertex_selection_ = polygon_.points.erase(v);
      if (vertex_selection_ == polygon_.points.end())
        vertex_selection_ = polygon_.points.begin();
      break;
    }
  }
}

void PolygonTool::renderPolygon() {
  ROS_ASSERT(polygon_node_);
  polygon_node_->removeAllChildren();  // Clear polygon visualization.

  const Ogre::ColourValue& c = kBlue;
  // Render vertices and edges.
  for (auto v = polygon_.points.begin(); v != polygon_.points.end(); ++v) {
    // Render vertices as spheres.
    rviz::Shape* sphere =
        new rviz::Shape(rviz::Shape::Sphere, scene_manager_, polygon_node_);
    sphere->setColor(c);
    sphere->setScale(Ogre::Vector3(kPtScale));
    Ogre::Vector3 p((double)(v->x), (double)(v->y), altitude_);
    sphere->setPosition(p);
    if (v == vertex_selection_) sphere->setColor(kGreen);

    // Render edges as lines.
    if (polygon_.points.size() < 2) continue;
    auto v_prev = v == polygon_.points.begin()
                      ? std::prev(polygon_.points.end())
                      : std::prev(v);
    Ogre::Vector3 start((double)(v_prev->x), (double)(v_prev->y), altitude_);
    rviz::Line* line = new rviz::Line(scene_manager_, polygon_node_);
    line->setColor(c);
    line->setPoints(start, p);
    if (v == vertex_selection_) line->setColor(kGreen);
  }
}

int PolygonTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel) {
  if (event->text() == "v") {
    nextVertex();
  } else if (event->text() == "c") {
    clearAll();
  } else if (event->matches(QKeySequence::InsertParagraphSeparator)) {
    publishPolygon();
  }

  renderPolygon();
  return Render;
}


void PolygonTool::nextVertex() {
  vertex_selection_ = std::next(vertex_selection_);
  if (vertex_selection_ == polygon_.points.end())
    vertex_selection_ = polygon_.points.begin();
}

void PolygonTool::clearAll() {
  polygon_ = geometry_msgs::Polygon();
  vertex_selection_ = polygon_.points.begin();
  altitude_ = kDefaultAltitude;
}
void PolygonTool::publishPolygon() {
  // Create ROS message.
  geometry_msgs::PolygonStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = context_->getFixedFrame().toStdString();
  msg.polygon = polygon_;
  polygon_pub_.publish(msg);

  ROS_INFO_STREAM("Publishing polygon");
}

/*void PolygonTool::increaseAltitude(rviz::ViewportMouseEvent& event) {
  if (event.shift()) {
    altitude_ += kSmallAltitudeDelta;
  } else if (event.control()) {
    altitude_ += kLargeAltitudeDelta;
  } else {
    altitude_ += kNormalAltitudeDelta;
  }

  renderPolygon();
}

void PolygonTool::decreaseAltitude(rviz::ViewportMouseEvent& event) {
  if (event.shift()) {
    altitude_ -= kSmallAltitudeDelta;
  } else if (event.control()) {
    altitude_ -= kLargeAltitudeDelta;
  } else {
    altitude_ -= kNormalAltitudeDelta;
  }

  renderPolygon();
}*/

}  // namespace rviz_polygon_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_polygon_tool::PolygonTool, rviz::Tool)
