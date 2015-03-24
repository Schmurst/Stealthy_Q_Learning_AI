//
// Simple implementation of a Q_learner for use with opensteer simple vehicle
// Sam Hayhurst 2015
//


#ifndef Q_LEARNER_INCLUDED
#define Q_LEARNER_INCLUDED

#include <vector>
#include "Fann\src\include\fann.h"


class Q_Learner{
private:
  float learning_discount;
  struct fann *ann;
  

public:
  Q_Learner(){
    init();
  }
  ~Q_Learner(){};

  enum actions { _SEEK, _EVADE, _HIDE };
  actions prev_action = _SEEK;

  struct worldState {
    int state; // 0 is game loop, 1 is tagged, 2 is at goal
    float goal_dist;
    float goal_angle;
    float enemy_dist;
    float enemy_angle;
    float hide_spot_dist;
    float hide_spot_angle;

    worldState(int seeker_state, float g_dist, float g_angle, float e_dist, float e_angle, float hs_dist, float hs_angle){
      state = seeker_state;
      goal_dist = g_dist;
      goal_angle = g_angle;
      enemy_dist = e_dist;
      enemy_angle = e_angle;
      hide_spot_dist = hs_dist;
      hide_spot_angle = hs_angle;
    }

    void print(){
      printf("g_dist: %f, g_angle: %f\n", goal_dist, goal_angle);
      printf("e_dist: %f, e_angle: %f\n", enemy_dist, enemy_angle);
      printf("ghs_dist: %f, sh_angle: %f\n", hide_spot_dist, hide_spot_angle);
    }

  };

  void init(){
    int num_inputs = 7;
    int num_layers = 3;
    int num_hidden = 6;
    int num_outputs = 3;
    // create fann network
    ann = fann_create_standard(num_layers, num_inputs, num_hidden, num_outputs);
    // set fail bit limit whatever that is
    fann_set_bit_fail_limit(ann, 0.01f);
    // set learning discount for Q-Learning
    learning_discount = 0.8f;
  }

  actions train_ann(const worldState* prev_ws, const worldState* curr_ws){
    float ann_training_values[3];
    fann_type* ann_current_values;
    ann_current_values = fann_run(ann, (fann_type*)&prev_ws);
    memcpy(ann_training_values, ann_current_values, sizeof(float) * 3);

    // calculate the reward value for previous state and action
    float reward = get_reward(prev_ws, curr_ws);
    // assign the reward to the training vlaues for the neural network
    ann_training_values[prev_action] = reward;
    // the training on the neural network
    fann_train(ann, (fann_type*)&prev_ws, ann_training_values);
    // assign the new best action to prev_action and return it
    prev_action = get_best_action(curr_ws);
    return prev_action;
  }

  // returns an action given a world state
  actions get_best_action(const worldState* state){
    int best_action = -1;
    float Q_max = -1000000;
    fann_type* out;

    out = fann_run(ann, (fann_type*)&state);
    for (int i = 0; i < 3; ++i){
      if (out[i] > Q_max){
        Q_max = out[i];
        best_action = i;
      }
    }

    if (best_action == 0 && rand() % 10 == 0) {
      printf("Random Action taken-----------------\n");
      best_action = rand() % 3;
    }

    return (actions) best_action;
  }

  // returns the maximum Q value for a given world state
  float get_Q_max(const worldState* state){
    float Q_max = -1000000;
    fann_type* out;

    out = fann_run(ann, (fann_type*)&state);
    for (int j = 0; j < 3; ++j)
    {
      if (*out > Q_max)
      {
        Q_max = *out;
      }
      ++out;
    }
    return Q_max;
  }

  // calculates the reward the Q Learner should recieve given two world states
  float get_reward(const worldState* prev_ws, const worldState* curr_ws){
    float reward = 0.0f;

    switch (curr_ws->state - prev_ws->state)
    {
    case 1:
      return 0.0f; // severe punishment for getting caught
      break;
    case 2:
      reward += 1.0f; // massive reward for success
    default:
      break;
    }

    // reward for approaching the goal
    if (curr_ws->goal_dist < prev_ws->goal_dist){
      reward += 0.05f;
    }
    else{
      reward -= 0.02f;
    }

    // reward for avoiding the enemy
    if (curr_ws->enemy_dist > prev_ws->enemy_dist){
      reward += 0.10f;
    }
    else{
      reward -= 0.400f;
    }

    // reward for moving towards hiding spot
    if (curr_ws->hide_spot_dist < prev_ws->hide_spot_dist){
      reward += 0.0f;
    }
    else{
      reward -= 0.0f;
    }

    // increment reward by learning discount * next best state
    reward += learning_discount * get_Q_max(curr_ws);

    return reward;
  }
};

#endif