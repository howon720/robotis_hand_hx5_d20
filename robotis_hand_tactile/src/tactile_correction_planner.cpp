#include "tactile_correction_planner.hpp"

namespace robotis_hand_tactile {

TactileCorrectionPlanner::TactileCorrectionPlanner(Controller& controller) : controller_(controller) {
}

void TactileCorrectionPlanner::start_correction(int finger_idx) {
  const auto maybe_decision = controller_.pick_correction(controller_.fingers_[finger_idx].cop);
  if (!maybe_decision.has_value()) {
    return;
  }

  if (controller_.hold_correction_stage_ == HoldCorrectionStage::Y_FIRST && !is_y_correction(maybe_decision->type)) {
    return;
  }

  if (controller_.hold_correction_stage_ == HoldCorrectionStage::X_SECOND && !is_x_correction(maybe_decision->type)) {
    return;
  }

  // max_min limit
  if (!is_at_joint_limit(finger_idx, maybe_decision->type)) {
    RCLCPP_INFO(controller_.get_logger(),
                "[%s] correction skipped: %s (joint limit)",
                controller_.fingers_[finger_idx].name.c_str(),
                correction_str(maybe_decision->type).c_str());
    return;
  }

  auto& plan = controller_.correction_plans_[finger_idx];
  plan.type = maybe_decision->type;
  plan.cost = maybe_decision->cost;
  plan.phase = 0;
  plan.active = true;

  switch (plan.type) {
  case CorrectionType::X_TOP:
  case CorrectionType::X_BOT:
  case CorrectionType::Y_LEFT:
  case CorrectionType::Y_RIGHT:
    plan.ticks_remaining = controller_.phase_step_;
    break;
  default:
    plan.active = false;
    break;
  }

  RCLCPP_INFO(controller_.get_logger(),
              "[%s] correction start: %s (cost=%.3f, cop_x_ratio=%.3f, "
              "cop_y_ratio=%.3f)",
              controller_.fingers_[finger_idx].name.c_str(),
              correction_str(plan.type).c_str(),
              plan.cost,
              controller_.fingers_[finger_idx].cop.cop_x_ratio,
              controller_.fingers_[finger_idx].cop.cop_y_ratio);
}

void TactileCorrectionPlanner::apply_correction(int finger_idx) {
  auto& plan = controller_.correction_plans_[finger_idx];
  const bool done = run_correction(finger_idx, plan);

  if (done) {
    plan = CorrectionPlan{};
    RCLCPP_INFO(controller_.get_logger(), "[%s] correction done", controller_.fingers_[finger_idx].name.c_str());
  }
}

bool TactileCorrectionPlanner::run_correction(int finger_idx, CorrectionPlan& plan) {
  if (!plan.active) {
    return true;
  }

  const double scale = controller_.finger_step_scale(finger_idx);

  switch (plan.type) {
  case CorrectionType::X_TOP: {
    const bool moved = controller_.correction_ik(finger_idx, true);
    plan.ticks_remaining--;
    return (!moved || plan.ticks_remaining <= 0);
  }

  case CorrectionType::X_BOT: {
    const bool moved = controller_.correction_ik(finger_idx, false);
    plan.ticks_remaining--;
    return (!moved || plan.ticks_remaining <= 0);
  }

  case CorrectionType::Y_LEFT: {
    if (plan.phase == 0) {
      controller_.release_234(finger_idx, controller_.y_corr_step_ * scale);
      plan.ticks_remaining--;
      if (plan.ticks_remaining <= 0) {
        plan.phase = 1;
        plan.ticks_remaining = controller_.phase_step_;
      }
      return false;
    }

    if (plan.phase == 1) {
      if (finger_idx == 0) {
        controller_.shift_thumb_y(finger_idx, controller_.shift_step_, controller_.shift_step_);
      } else {
        controller_.shift_joint1(finger_idx, -controller_.shift_step_);
      }

      plan.ticks_remaining--;
      if (plan.ticks_remaining <= 0) {
        plan.phase = 2;
        plan.ticks_remaining = controller_.phase_step_;
      }
      return false;
    }

    if (plan.phase == 2) {
      controller_.grasp_234(finger_idx, controller_.y_corr_step_ * scale);
      plan.ticks_remaining--;
      return (plan.ticks_remaining <= 0);
    }
    return true;
  }

  case CorrectionType::Y_RIGHT: {
    if (plan.phase == 0) {
      controller_.release_234(finger_idx, controller_.y_corr_step_ * scale);
      plan.ticks_remaining--;
      if (plan.ticks_remaining <= 0) {
        plan.phase = 1;
        plan.ticks_remaining = controller_.phase_step_;
      }
      return false;
    }

    if (plan.phase == 1) {
      if (finger_idx == 0) {
        // thumb: joint1, joint2 둘 다 -- 방향
        controller_.shift_thumb_y(finger_idx, -controller_.shift_step_, -controller_.shift_step_);
      } else {
        controller_.shift_joint1(finger_idx, +controller_.shift_step_);
      }

      plan.ticks_remaining--;
      if (plan.ticks_remaining <= 0) {
        plan.phase = 2;
        plan.ticks_remaining = controller_.phase_step_;
      }
      return false;
    }

    if (plan.phase == 2) {
      controller_.grasp_234(finger_idx, controller_.y_corr_step_ * scale);
      plan.ticks_remaining--;
      return (plan.ticks_remaining <= 0);
    }
    return true;
  }

  default:
    return true;
  }
}

bool TactileCorrectionPlanner::is_at_joint_limit(int finger_idx, CorrectionType type) const {

  const auto& finger = controller_.fingers_[finger_idx];
  const double eps = 1e-6;
  const bool is_thumb = (finger_idx == 0);

  auto max_limit = [&](int joint_idx) {
    return finger.current_joint_targets[joint_idx] < finger.joint_max[joint_idx] - eps;
  };

  auto min_limit = [&](int joint_idx) {
    return finger.current_joint_targets[joint_idx] > finger.joint_min[joint_idx] + eps;
  };

  switch (type) {
  case CorrectionType::Y_LEFT:
    if (is_thumb) {
      return max_limit(0) || max_limit(1);
    }
    return min_limit(0);

  case CorrectionType::Y_RIGHT:
    if (is_thumb) {
      return min_limit(0) || min_limit(1);
    }
    return max_limit(0);

  case CorrectionType::X_TOP:
  case CorrectionType::X_BOT:
    return (min_limit(1) && max_limit(1)) || (min_limit(2) && max_limit(2)) || (min_limit(3) && max_limit(3));

  default:
    return false;
  }
}

bool TactileCorrectionPlanner::is_y_correction(CorrectionType type) const {
  return type == CorrectionType::Y_LEFT || type == CorrectionType::Y_RIGHT;
}

bool TactileCorrectionPlanner::is_x_correction(CorrectionType type) const {
  return type == CorrectionType::X_TOP || type == CorrectionType::X_BOT;
}

bool TactileCorrectionPlanner::correction_blocked(HoldCorrectionStage stage) const {
  for (int i = 0; i < Controller::fingers_num; ++i) {
    const auto maybe_decision = controller_.pick_correction(controller_.fingers_[i].cop);
    if (!maybe_decision.has_value()) {
      continue;
    }

    if (stage == HoldCorrectionStage::Y_FIRST && !is_y_correction(maybe_decision->type)) {
      continue;
    }

    if (stage == HoldCorrectionStage::X_SECOND && !is_x_correction(maybe_decision->type)) {
      continue;
    }

    if (is_at_joint_limit(i, maybe_decision->type)) {
      return false;
    }
  }
  return true;
}

std::string TactileCorrectionPlanner::correction_str(CorrectionType t) const {
  switch (t) {
  case CorrectionType::NONE:
    return "NONE";
  case CorrectionType::X_TOP:
    return "X_TOP";
  case CorrectionType::X_BOT:
    return "X_BOT";
  case CorrectionType::Y_LEFT:
    return "Y_LEFT";
  case CorrectionType::Y_RIGHT:
    return "Y_RIGHT";
  default:
    return "UNKNOWN";
  }
}

} // namespace robotis_hand_tactile