/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef GO1_FSM_HPP
#define GO1_FSM_HPP

#include "fsm_a1.hpp"

namespace go1_fsm
{
using RLFSMStatePassive = a1_fsm::RLFSMStatePassive;
using RLFSMStateGetUp = a1_fsm::RLFSMStateGetUp;
using RLFSMStateGetDown = a1_fsm::RLFSMStateGetDown;
using RLFSMStateRLLocomotion = a1_fsm::RLFSMStateRLLocomotion;
} // namespace go1_fsm

class GO1FSMFactory : public FSMFactory
{
public:
    explicit GO1FSMFactory(const std::string &initial) : initial_state_(initial) {}

    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<go1_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<go1_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<go1_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRLLocomotion")
            return std::make_shared<go1_fsm::RLFSMStateRLLocomotion>(rl);
        return nullptr;
    }

    std::string GetType() const override { return "go1"; }

    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRLLocomotion"};
    }

    std::string GetInitialState() const override { return initial_state_; }

private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(GO1FSMFactory, "RLFSMStatePassive")

#endif // GO1_FSM_HPP
