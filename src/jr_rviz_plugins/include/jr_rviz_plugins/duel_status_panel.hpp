// Панель RViz со статусом соревнования.
//
// Показывает исход, расстояние атакующего до синей зоны по прямой и вдоль
// пути, а также расстояние между роботами. Исход выделяется цветом, чтобы
// конец соревнования был заметен сразу.

#ifndef JR_RVIZ_PLUGINS__DUEL_STATUS_PANEL_HPP_
#define JR_RVIZ_PLUGINS__DUEL_STATUS_PANEL_HPP_

#include <memory>
#include <string>

#include <QLabel>
#include <QLineEdit>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "jr_msgs/msg/duel_status.hpp"

namespace jr_rviz_plugins
{

class DuelStatusPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit DuelStatusPanel(QWidget * parent = nullptr);

  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private Q_SLOTS:
  void updateTopic();

private:
  void onStatus(const jr_msgs::msg::DuelStatus::ConstSharedPtr message);

  // Пока сообщений нет, показываем прочерки, а не нули: ноль означал бы,
  // что робот уже в зоне.
  void showWaiting();

  QLineEdit * topic_edit_{nullptr};
  QLabel * result_label_{nullptr};
  QLabel * goal_euclidean_label_{nullptr};
  QLabel * goal_path_label_{nullptr};
  QLabel * robots_label_{nullptr};

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<jr_msgs::msg::DuelStatus>::SharedPtr subscription_;
  std::string topic_{"/duel/status"};
};

}  // namespace jr_rviz_plugins

#endif  // JR_RVIZ_PLUGINS__DUEL_STATUS_PANEL_HPP_
