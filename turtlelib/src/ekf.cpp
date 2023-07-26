#include "turtlelib/ekf.hpp"
#include <cstdio>
#include <armadillo>

namespace turtlelib
{
    EKF::EKF()
    : state{arma::vec ((2*n + 3), arma::fill::zeros)},
      state_est{arma::vec((2*n + 3), arma::fill::zeros)},
      prev_config{0.0,0.0,0.0},
      curr_config{0.0,0.0,0.0}
      {
        arma::mat sigma_q0(3,3, arma::fill::zeros);
        arma::mat zeros_1 (3,2*n , arma::fill::zeros);
        arma::mat zeros_2(2*n,3, arma::fill::zeros);
        arma::mat sigma_m0(2*n,2*n, arma::fill::eye);
        sigma_m0*=9999.0;
        auto cov_top = arma::join_rows( sigma_q0, zeros_1 );
        auto cov_bottom= arma::join_rows( zeros_2, sigma_m0 );
        cov = arma::join_cols( cov_top, cov_bottom );
      }


    void EKF::setEKF_config(double x, double y, double theta)
    {
      curr_config.x = x;
      curr_config.y = y;
      curr_config.theta = theta;
    }

    arma::vec EKF::getState()
    {
        return state_est;
    }

    arma::mat EKF::calc_Amat()
    {
        double delta_x = curr_config.x - prev_config.x;
        double delta_y = curr_config.y - prev_config.y;

        arma::mat I(2*n + 3,2*n + 3, arma::fill::eye);
        I(1,0) = -delta_y;
        I(2,0) = delta_x;

        return I;
    }

    int EKF::data_association(double center_x, double center_y)
    {

      int l = num_obstacles + 1;
  
      double rel_x = center_x; 
      double rel_y = center_y;
      double r_c = sqrt(pow(rel_x,2) + pow(rel_y,2));
      double phi = atan2(rel_y,rel_x);

      arma::vec temp_est = state_est;
      temp_est(3+(2*num_obstacles)+1) = temp_est(1) + r_c*cos(phi + temp_est(0));
      temp_est(3+(2*num_obstacles)+2) = temp_est(2) + r_c*sin(phi + temp_est(0));
      arma::vec z_meas;
      z_meas = {r_c, phi};
      std::vector<double> m_list;

      for (int i = 0; i < num_obstacles+1; i++)
      {
        arma::vec zt_meas, zdiff;
        double x_dist = temp_est(3+(2*i)) - temp_est(1);
        double y_dist = temp_est(4+(2*i)) - temp_est(2);
        double est_dist = pow(x_dist,2) + pow(y_dist,2);
        zt_meas = {sqrt(est_dist) , normalize_angle(atan2(y_dist, x_dist) - temp_est(0))};

        arma::mat subh_1, subh_2, subh_3, subh_4;

        subh_1 = {{0.0,(-x_dist)/sqrt(est_dist),(-y_dist)/sqrt(est_dist)},
                    {-1.0,y_dist/est_dist, -x_dist/est_dist }};

        subh_2 = arma::mat(2,2*i , arma::fill::zeros);

        subh_3 = {{x_dist/sqrt(est_dist),y_dist/sqrt(est_dist)},
                            {-y_dist/est_dist, x_dist/est_dist }};

        subh_4 = arma::mat(2,(2*n) - (2*(i+1)) , arma::fill::zeros);

        H= arma::join_rows( subh_1, subh_2, subh_3, subh_4);

        double R_noise = 0.1;  
        arma::mat R{arma::mat(2,2, arma::fill::eye)};
        R*=R_noise;

        arma::mat phi = (H*cov_est* H.t())  + R;

        zdiff = z_meas - zt_meas;
        zdiff(1) = normalize_angle(zdiff(1));

        arma::mat m_dist = (zdiff).t() * phi.i() * (zdiff);

        m_list.push_back(m_dist(0));
    
      }

      double m_dist_thresh = m_list.at(m_list.size()-1);

      bool new_obstacle = true;


      for (size_t i = 0; i < m_list.size(); i++)
      {
        if (m_list.at(i) < m_dist_thresh)
        {
          new_obstacle = false;
          m_dist_thresh = m_list.at(i);
          l= i;
        }
      }

      if (new_obstacle == true)
      {
        num_obstacles++;
      }
      return l;
    }

    void EKF::predict_state()
    {
        // State prediction 
        arma::vec m0((2*n), arma::fill::zeros);
        arma::vec delta_q{normalize_angle(curr_config.theta - prev_config.theta), (curr_config.x - prev_config.x), curr_config.y - prev_config.y};
        arma::vec deltaq_vec = arma::join_cols( delta_q, m0);

        state_est += deltaq_vec;  
        prev_config = curr_config;
        
        // Create Q matrix
        double Q_noise = 0.1;
        arma::mat Q(3,3, arma::fill::eye);
        arma::mat zeros_1 (3,2*n , arma::fill::zeros);
        arma::mat zeros_2(2*n,3, arma::fill::zeros);
        arma::mat sigma_m0(2*n,2*n, arma::fill::zeros);
        Q*=Q_noise;
        arma::mat Q_top = arma::join_rows( Q, zeros_1 );
        arma::mat Q_bottom = arma::join_rows( zeros_2, sigma_m0 );
        arma::mat Q_bar = arma::join_cols( Q_top, Q_bottom );   // Process noise

        // Covariance prediction 
        arma::mat A = calc_Amat();
        cov_est = (A*cov*A.t()) + Q_bar;
    }

    void EKF::update_state(double cx, double cy, int j)
    {
        double rel_x = cx; 
        double rel_y = cy; 
        double r_c = sqrt(pow(rel_x,2) + pow(rel_y,2));
        double phi = atan2(rel_y,rel_x);

        if (obstacle_set.find(j) == obstacle_set.end())
        {
            obstacle_set.insert(j);
            state_est(3+(2*j)) = state_est(1) + r_c*cos(phi + state_est(0));
            state_est(3+(2*j)+1) = state_est(2) + r_c*sin(phi + state_est(0));
        }
        arma::vec z_meas, zt_meas, zdiff;
        z_meas = {r_c, phi};
        double x_dist = state_est(3+(2*j)) - state_est(1);
        double y_dist = state_est(4+(2*j)) - state_est(2);
        double est_dist = pow(x_dist,2) + pow(y_dist,2);
        zt_meas = {sqrt(est_dist) , normalize_angle(atan2(y_dist, x_dist) - state_est(0))};

        arma::mat subh_1, subh_2, subh_3, subh_4;

        subh_1 = {{0.0,(-x_dist)/sqrt(est_dist),(-y_dist)/sqrt(est_dist)},
                    {-1.0,y_dist/est_dist, -x_dist/est_dist }};

        subh_2 = arma::mat(2,2*j , arma::fill::zeros);

        subh_3 = {{x_dist/sqrt(est_dist),y_dist/sqrt(est_dist)},
                            {-y_dist/est_dist, x_dist/est_dist }};

        subh_4 = arma::mat(2,(2*n) - (2*(j+1)) , arma::fill::zeros);

        H= arma::join_rows( subh_1, subh_2, subh_3, subh_4);
        
        // 
        double R_noise = 0.1;  
        arma::mat R{arma::mat(2,2, arma::fill::eye)};
        R*=R_noise;
        K = (cov_est*H.t())*(((H*cov_est*H.t()) + R).i());
        
        // Update the state and covariance predictions
        zdiff = z_meas - zt_meas;
        zdiff(1) = normalize_angle(zdiff(1));

        state = state_est + K*(zdiff);
        state(0) = normalize_angle(state(0));

        arma::mat I(2*n + 3,2*n + 3, arma::fill::eye);
        cov = ((I-(K*H))) * cov_est;

        state_est = state;
        cov_est = cov;

    }
} 
