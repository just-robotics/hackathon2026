#include "jr_rviz_plugins/duel_status_panel.hpp"

#include <cmath>

#include <QFormLayout>
#include <QFont>
#include <QVBoxLayout>

#include <rviz_common/display_context.hpp>

namespace jr_rviz_plugins
{

namespace
{

// Прочерк вместо числа, пока данных нет.
const char * kNoData = "—";

QString formatMetres(double value)
{
  if (std::isnan(value)) {
    return QString(kNoData);
  }
  return QString::number(value, 'f', 2) + " м";
}

}  // namespace

DuelStatusPanel::DuelStatusPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  topic_edit_ = new QLineEdit(QString::fromStdString(topic_));
  connect(topic_edit_, SIGNAL(editingFinished()), this, SLOT(updateTopic()));

  result_label_ = new QLabel();
  QFont font = result_label_->font();
  font.setPointSize(font.pointSize() + 4);
  font.setBold(true);
  result_label_->setFont(font);
  result_label_->setAlignment(Qt::AlignCenter);

  goal_euclidean_label_ = new QLabel();
  goal_path_label_ = new QLabel();
  robots_label_ = new QLabel();

  auto * form = new QFormLayout();
  form->addRow("Топик", topic_edit_);
  form->addRow("До зоны, прямая", goal_euclidean_label_);
  form->addRow("До зоны, по пути", goal_path_label_);
  form->addRow("Между роботами", robots_label_);

  auto * layout = new QVBoxLayout();
  layout->addWidget(result_label_);
  layout->addLayout(form);
  setLayout(layout);

  showWaiting();
}

void DuelStatusPanel::showWaiting()
{
  result_label_->setText("нет данных");
  result_label_->setStyleSheet("color: gray;");
  goal_euclidean_label_->setText(kNoData);
  goal_path_label_->setText(kNoData);
  robots_label_->setText(kNoData);
}

void DuelStatusPanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  updateTopic();
}

void DuelStatusPanel::updateTopic()
{
  topic_ = topic_edit_->text().toStdString();

  if (topic_.empty()) {
    subscription_.reset();
    showWaiting();
    return;
  }

  // Судья публикует с RELIABLE и глубиной 1; та же политика нужна и здесь,
  // иначе подписка не совпадёт по QoS и сообщения не дойдут.
  subscription_ = node_->create_subscription<jr_msgs::msg::DuelStatus>(
    topic_, rclcpp::QoS(1).reliable(),
    [this](const jr_msgs::msg::DuelStatus::ConstSharedPtr message) {
      onStatus(message);
    });

  showWaiting();
}

void DuelStatusPanel::onStatus(
  const jr_msgs::msg::DuelStatus::ConstSharedPtr message)
{
  if (message->result == jr_msgs::msg::DuelStatus::ATTACKER_WON) {
    result_label_->setText("победил атакующий");
    result_label_->setStyleSheet("color: #d32f2f;");
  } else if (message->result == jr_msgs::msg::DuelStatus::DEFENDER_WON) {
    result_label_->setText("победил защитник");
    result_label_->setStyleSheet("color: #1976d2;");
  } else {
    result_label_->setText("идёт");
    result_label_->setStyleSheet("");
  }

  goal_euclidean_label_->setText(
    formatMetres(message->goal_distance_euclidean));
  goal_path_label_->setText(formatMetres(message->goal_distance_path));
  robots_label_->setText(formatMetres(message->robots_distance));
}

void DuelStatusPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("Topic", QString::fromStdString(topic_));
}

void DuelStatusPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString topic;
  if (config.mapGetString("Topic", &topic)) {
    topic_edit_->setText(topic);
    updateTopic();
  }
}

}  // namespace jr_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jr_rviz_plugins::DuelStatusPanel, rviz_common::Panel)
