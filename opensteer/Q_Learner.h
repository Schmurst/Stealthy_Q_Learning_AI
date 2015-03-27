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
  float alpha;
  struct fann *ann;
  float gamma;


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
    float enemy_facing; // direction the enemy is facing with regards to the seeker
    float hide_spot_dist;
    float hide_spot_angle;

    worldState(int seeker_state, float g_dist, float g_angle, float e_dist, float e_angle, float e_facing, float hs_dist, float hs_angle){
      state = seeker_state;
      goal_dist = g_dist;
      goal_angle = g_angle;
      enemy_dist = e_dist;
      enemy_angle = e_angle;
      enemy_facing = e_facing;
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
    int num_inputs = 8;
    int num_layers = 3;
    int num_hidden = 7;
    int num_outputs = 3;

    // create fann network from file
    ann = fann_create_from_file("neural_network.txt");
    if (ann == NULL){
      ann = fann_create_standard(num_layers, num_inputs, num_hidden, num_outputs);
    }

    // set fail bit limit whatever that is
    fann_set_bit_fail_limit(ann, 0.01f);

    // set learning discount for Q-Learning
    alpha = 0.9f;
    gamma = 0.7f;
  }

  actions train_ann(const worldState* prev_ws, const worldState* curr_ws){
    float ann_training_values[3];
    fann_type* ann_current_values;
    ann_current_values = fann_run(ann, (fann_type*)&prev_ws);
    memcpy(ann_training_values, ann_current_values, sizeof(float) * 3);

    // calculate the reward value for previous state and action
    float reward = get_reward(prev_ws, curr_ws);
    // assign the reward to the training vlaues for the neural network
    printf("Action Reward: %f\n", reward);
    printf("Current Value: %f\n", ann_training_values[prev_action]);
    float old_value = ann_training_values[prev_action];
    ann_training_values[prev_action] = (1 - alpha) * old_value + alpha * (reward + gamma * get_Q_max(curr_ws));
    // the training on the neural network
    fann_train(ann, (fann_type*)&prev_ws, ann_training_values);
    // assign the new best action to prev_action and return it
    prev_action = get_best_action(curr_ws);

    fann_save(ann, "neural_network.txt");
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

    if (rand() % 10 < 1) {
      printf("Random Action taken\n");
      best_action = rand() % 3;
    }

    return (actions)best_action;
  }

  // returns the maximum Q value for a given world state
  float get_Q_max(const worldState* state){
    float Q_max = -1.0f;
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

    // GOAL REWARD
    if (curr_ws->goal_dist < prev_ws->goal_dist + 1.0f){
      reward += 0.2f;
    }
    else if (curr_ws->goal_dist > prev_ws->goal_dist + 1.0f){
      reward += 0.005f;
    }
    else{
      reward += 0.05;
    }

    // ENEMY REWARD
    if (curr_ws->enemy_dist < prev_ws->enemy_dist + 1.0f){
      reward += 0.005f;
    }
    else if (curr_ws->enemy_dist > prev_ws->enemy_dist + 1.0f){
      reward += 0.25f;
    }
    else{
      reward += 0.1f;
    }

    // ENEMY FACING REWARD
    if (curr_ws->enemy_facing < prev_ws->enemy_facing && curr_ws->enemy_facing < 0.52f){
      reward += 0.005f;
    }
    else if (curr_ws->enemy_facing > prev_ws->enemy_facing + 0.26f){
      reward += 0.01f;
    }

    // ENEMY ANGLE REWARD
    if (curr_ws->enemy_angle > prev_ws->enemy_angle + 0.26f){
      reward += 0.01f;
    }
    else if (curr_ws->enemy_facing < prev_ws->enemy_facing + 0.26f){
      reward += 0.001f;
    }

    // HIDE DIST REWARD
    if (curr_ws->hide_spot_dist < prev_ws->hide_spot_dist + 1.0f){
      reward += 0.1f;
    }
    else if (curr_ws->hide_spot_dist < prev_ws->hide_spot_dist + 1.0f){
      reward += 0.001f;
    }


    /*
    // GOAL RELATED REWARD ---------------------
    // reward for approaching the goal
    if (curr_ws->goal_dist < prev_ws->goal_dist){
      reward += 0.5f - (1 / 30)*curr_ws->goal_dist;
    }
    else{
      reward -= 0.5f - (1 / 30)*curr_ws->goal_dist;
    }

    // ENEMY RELATED REWARD ---------------------
    // reward for avoiding the enemy
    if (curr_ws->enemy_dist < 10.0f){
      if (curr_ws->enemy_dist > prev_ws->enemy_dist){
        reward += 0.8f - 0.03f*curr_ws->goal_dist;
      }
      else{
        reward -= 0.8f - 0.03f*curr_ws->goal_dist;
      }
    }
    else{
      if (curr_ws->enemy_dist > prev_ws->enemy_dist){
        reward += 0.4f - 0.02f*curr_ws->goal_dist;
      }
      else{
        reward -= 0.4f - 0.02f*curr_ws->goal_dist;
      }
    }

    // reward for moving past enemy, punish for moving towards
    if (curr_ws->enemy_angle < curr_ws->enemy_angle){
      reward += 0.005f;
    }
    else{
      reward -= 0.01f;
    }

    // punish for being inside enemy sign arc
    if (abs(curr_ws->enemy_facing) < 0.52f){
      reward -= 0.1f;
    }

    // HIDE RELATED REWARD ---------------------
    // reward for moving towards hiding spot
    if (curr_ws->hide_spot_dist < prev_ws->hide_spot_dist){
      reward += 0.05f;
    }
    else{
      reward -= 0.025f;
    }

    // reward for turning towards hiding spot
    if (curr_ws->hide_spot_angle < prev_ws->hide_spot_angle){
      reward += 0.0025f;
    }
    else {
      reward -= 0.00125f;
    }

    // reward to avoid sitting outside the 'play area'
    if (curr_ws->goal_dist < prev_ws->goal_dist && curr_ws->goal_dist > 30.0f){
      reward = 1.0f;
    }
    else{
      reward = 0.0f;
    }

    */
    
    return reward;
  }
};

#endif