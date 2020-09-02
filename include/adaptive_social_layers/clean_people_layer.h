#ifndef ADAPTIVE_LAYER_H_
#define ADAPTIVE_LAYER_H_
#include <ros/ros.h>
#include <adaptive_social_layers/social_layer.h>
#include <dynamic_reconfigure/server.h>
#include <adaptive_social_layers/CleanPeopleLayerConfig.h>



namespace adaptive_social_layers
{
  class CleanPeopleLayer : public SocialLayer
  {
    public:
      CleanPeopleLayer() { layered_costmap_ = NULL; }

      virtual void onInitialize();
      virtual void updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y);
      virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

    protected:
      void configure(CleanPeopleLayerConfig &config, uint32_t level);
      double clcdis_;
      dynamic_reconfigure::Server<CleanPeopleLayerConfig>* server_;
      dynamic_reconfigure::Server<CleanPeopleLayerConfig>::CallbackType f_;
  };
};


#endif
