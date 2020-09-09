#ifndef ADAPTIVE_LAYER_H_
#define ADAPTIVE_LAYER_H_
#include <ros/ros.h>
#include <adaptive_social_layers/social_layer.h>
#include <dynamic_reconfigure/server.h>
#include <adaptive_social_layers/AdaptiveLayerConfig.h>



double gaussian(double x, double y, double x0, double y0, double A, double varx, double vary, double skew);
double get_radius(double cutoff, double A, double var);
double distance(double x1, double y1, double x2, double y2);

//Human Body Dimensions top view in m
#define HUMAN_Y  0.45
#define HUMAN_X  0.20

namespace adaptive_social_layers
{
  class AdaptiveLayer : public SocialLayer
  {
    public:
      AdaptiveLayer() { layered_costmap_ = NULL; }

      virtual void onInitialize();
      virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
      

    protected:
      void configure(AdaptiveLayerConfig &config, uint32_t level);
      double cutoff_, amplitude_, factor_;
      dynamic_reconfigure::Server<AdaptiveLayerConfig>* server_;
      dynamic_reconfigure::Server<AdaptiveLayerConfig>::CallbackType f_;
  };
};


#endif
