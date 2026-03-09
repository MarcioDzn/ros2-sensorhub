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
            plt::clf(); 

            for (auto& [name, series] : series_map) {
                plt::named_plot(name, series.x, series.y);
            }
            plt::legend();
            plt::title("Window " + std::to_string(window_id));
            
            // O draw() força a atualização da janela específica
            plt::draw();
        }
        
        // A pausa é o que permite que o X11 processe os eventos da janela
        plt::pause(0.001);
    }

    void show() {
        plt::show();
    }

private:
    struct Series {
        std::vector<double> x;
        std::vector<double> y;
    };
    std::map<int, std::map<std::string, Series>> data_;
};

#endif