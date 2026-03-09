#ifndef PLOTTER_HPP
#define PLOTTER_HPP

#include <string>
#include <vector>
#include <map>
#include <iostream>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

class Plotter
{
public:
    Plotter() {
        // Nada aqui, conforme solicitado.
    }

    void add_data(int window_id, const std::string& series_name, double x, double y) {
        data_[window_id][series_name].x.push_back(x);
        data_[window_id][series_name].y.push_back(y);
    }

    void plot() {
        plt::ion(); 

        for (auto& [window_id, series_map] : data_) {
            plt::figure(window_id);
<<<<<<< HEAD
            plt::clf();
=======
            plt::clf(); 
>>>>>>> b73daf9d14fd63e3de9143ec9fa179b21030ebe7

            for (auto& [name, series] : series_map) {
                plt::named_plot(name, series.x, series.y);
            }
<<<<<<< HEAD

            plt::legend();

            std::string title = window_names_[window_id];
            if(title.empty())
                title = "Window " + std::to_string(window_id);

            plt::title(title);

            plt::draw();
        }

        plt::pause(0.001);
    }
    
    void set_window_name(int window_id, const std::string& name){
        window_names_[window_id] = name;
    }
=======
            plt::legend();
            plt::title("Window " + std::to_string(window_id));
            
            // O draw() força a atualização da janela específica
            plt::draw();
        }
        
        // A pausa é o que permite que o X11 processe os eventos da janela
        plt::pause(0.001);
    }
>>>>>>> b73daf9d14fd63e3de9143ec9fa179b21030ebe7

    void show() {
        plt::show();
    }

private:
    struct Series {
        std::vector<double> x;
        std::vector<double> y;
    };
    std::map<int, std::map<std::string, Series>> data_;
<<<<<<< HEAD
    std::map<int, std::string> window_names_;
};

#endif
=======
};

#endif
>>>>>>> b73daf9d14fd63e3de9143ec9fa179b21030ebe7
