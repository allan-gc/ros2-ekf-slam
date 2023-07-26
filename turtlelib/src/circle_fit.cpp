#include "turtlelib/circle_fit.hpp"
#include <numeric>
#include <armadillo>

namespace turtlelib
{
    Circle circle_fit(std::vector<Vector2D> cluster)
    {
        arma::vec x_coords(cluster.size());
        arma::vec y_coords(cluster.size());
        arma::vec z_vec(cluster.size());
        double xsum = 0.0;
        double ysum = 0.0;
        double zsum = 0.0;

        // int n = cluster.size();
        
        for (size_t i = 0; i < cluster.size(); i++)
        {
            x_coords(i) = (cluster.at(i).x);
            y_coords(i) = (cluster.at(i).y);
            xsum += x_coords(i);
            ysum += y_coords(i);

        }

        double x_avg = xsum / cluster.size();
        double y_avg = ysum / cluster.size();

        for (size_t i = 0; i < cluster.size(); i++)
        {
            x_coords(i) -= x_avg;
            y_coords(i) -= y_avg;
            double zi = std::pow(x_coords.at(i) ,2) + std::pow(y_coords.at(i) ,2);
            z_vec(i) = zi;
            zsum += z_vec(i);
        }

        double z_avg = zsum /cluster.size();

        // compute Z

        arma::mat Z(cluster.size(),4);

        for (long unsigned int  i = 0; i < cluster.size(); i++)
        {
            Z(i,0) = z_vec(i);
            Z(i,1) = x_coords(i);
            Z(i,2) = y_coords(i);
            Z(i,3) = 1.0;
        }


        // Compute H and H inv
        arma::vec H_1{8.0*z_avg, 0,0,2};
        arma::vec H_2{0,1,0,0};
        arma::vec H_3{0,0,1,0};
        arma::vec H_4{2,0,0,0};
        arma::mat H = arma::join_rows(H_1,H_2, H_3, H_4);


        arma::vec H_1_inv{0, 0,0,0.5};
        arma::vec H_2_inv{0,1,0,0};
        arma::vec H_3_inv{0,0,1,0};
        arma::vec H_4_inv{0.5,0,0,-2.0 * z_avg};
        arma::mat H_inv = arma::join_rows(H_1_inv,H_2_inv, H_3_inv, H_4_inv);
        

        //SVD of Z
        arma::mat U(4,4,arma::fill::zeros);
        arma::vec s(4);
        arma::mat V(4,4,arma::fill::zeros);
        arma::svd(U,s,V,Z);

        arma::mat sigma = arma::diagmat(s);
 
        arma::vec A;

        if (s(3) < 1e-12)
        {
            A = V.col(3);
        }
        else
        {
            arma::mat Y = V*sigma*V.t();

            // Calculate Q 
            arma::mat Q = Y * H_inv * Y;

            arma::cx_vec eig_val;
            arma::cx_mat eig_vec;

            arma::eig_gen(eig_val, eig_vec, Q);

            double min_val = 1000.0;
            int min_val_index = 0;
            for (unsigned int i = 0; i < eig_val.n_elem ;i++)
            {
                if (eig_val(i).real() < min_val && eig_val(i).real() > 0.0)
                {
                    min_val = eig_val(i).real();
                    min_val_index = i;
                }
            }

            arma::vec A_star{eig_vec(0,min_val_index).real(), eig_vec(1,min_val_index).real(), eig_vec(2,min_val_index).real(),eig_vec(3,min_val_index).real()};
            A = Y.i() * A_star;
        }

        double a = -A(1) / (2.0 * A(0));
        double b = -A(2) / (2.0 * A(0));

        double r_val = (pow(A(1),2) + pow(A(2),2) - (4.0*A(0)*A(3))) / (4.0*pow(A(0),2));
        double R = sqrt(r_val);

        Vector2D center{a+x_avg, b+y_avg};
        Circle circle{center, R};

        return circle;
    }
};

