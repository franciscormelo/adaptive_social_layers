#include <adaptive_social_layers/clean_people_layer.h>
#include <math.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>


PLUGINLIB_EXPORT_CLASS(adaptive_social_layers::CleanPeopleLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;



namespace adaptive_social_layers
{
    void CleanPeopleLayer::onInitialize()
    {
        SocialLayer::onInitialize();
        ros::NodeHandle nh("~/" + name_), g_nh;
        server_ = new dynamic_reconfigure::Server<CleanPeopleLayerConfig>(nh);
        f_ = boost::bind(&CleanPeopleLayer::configure, this, _1, _2);
        server_->setCallback(f_);
    }

    void CleanPeopleLayer::updateBoundsFromPeople(double* min_x, double* min_y, double* max_x, double* max_y)
    {
        std::list<group_msgs::Person>::iterator p_it;

        for(p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it){
            group_msgs::Person person = *p_it;

            *min_x = std::min(*min_x, person.position.x - clcdis_ );
            *min_y = std::min(*min_y, person.position.y - clcdis_ );
            *max_x = std::max(*max_x, person.position.x + clcdis_ );
            *max_y = std::max(*max_y, person.position.y + clcdis_ );

        }
    }

    void CleanPeopleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j){
        boost::recursive_mutex::scoped_lock lock(lock_);
        if(!enabled_) return;

        if( people_list_.people.size() == 0 )
          return;


        std::list<group_msgs::Person>::iterator p_it;
        costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
        double res = costmap->getResolution();
        int size = transformed_people_.size();

        for(p_it = transformed_people_.begin(); p_it != transformed_people_.end(); ++p_it){
            group_msgs::Person person = *p_it;
            double angle = person.orientation;
            double base;
            double point ;
            

            base = clcdis_;
            point = clcdis_;
            
            unsigned int width = std::max(1, int( (base + point) / res )),
                          height = std::max(1, int( (base + point) / res ));

            double cx = person.position.x, cy = person.position.y;

            double ox, oy;
            if(sin(angle)>0)
                oy = cy - base;
            else
                oy = cy + (point-base) * sin(angle) - base;

            if(cos(angle)>=0)
                ox = cx - base;
            else
                ox = cx + (point-base) * cos(angle) - base;


            int dx, dy;
            costmap->worldToMapNoBounds(ox, oy, dx, dy);

            int start_x = 0, start_y=0, end_x=width, end_y = height;
            if(dx < 0)
                start_x = -dx;
            else if(dx + width > costmap->getSizeInCellsX())
                end_x = std::max(0, (int)costmap->getSizeInCellsX() - dx);

            if((int)(start_x+dx) < min_i)
                start_x = min_i - dx;
            if((int)(end_x+dx) > max_i)
                end_x = max_i - dx;

            if(dy < 0)
                start_y = -dy;
            else if(dy + height > costmap->getSizeInCellsY())
                end_y = std::max(0, (int) costmap->getSizeInCellsY() - dy);

            if((int)(start_y+dy) < min_j)
                start_y = min_j - dy;
            if((int)(end_y+dy) > max_j)
                end_y = max_j - dy;

            double bx = ox + res / 2,
                   by = oy + res / 2;
            for(int i=start_x;i<end_x;i++){
                for(int j=start_y;j<end_y;j++){
                    costmap->setCost(i+dx, j+dy, 0);

              }
            }


        }
    }

    void CleanPeopleLayer::configure(CleanPeopleLayerConfig &config, uint32_t level) {
        people_keep_time_ = ros::Duration(config.keep_time);
        enabled_ = config.enabled;
        clcdis_ = config.clcdis;
    }


};