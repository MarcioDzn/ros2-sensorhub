#ifndef PLOTTER_HPP
#define PLOTTER_HPP

#include <string>
#include <vector>
#include <map>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

class Plotter
{
public:
    Plotter(int rows = 1, int cols = 1)
        : rows_(rows), cols_(cols)
    {
        plt::ion(); // modo interativo
    }

    // Adiciona dado a um subplot específico
    void add_data(int subplot_index,
                  const std::string& series_name,
                  double x,
                  double y)
    {
        data_[subplot_index][series_name].x.push_back(x);
        data_[subplot_index][series_name].y.push_back(y);
    }

    void plot()
    {
        plt::clf();

        for (auto& [subplot_index, series_map] : data_)
        {
            plt::subplot(rows_, cols_, subplot_index);

            for (auto& [name, series] : series_map)
            {
                plt::named_plot(name, series.x, series.y);
            }

            plt::legend();
        }

        plt::pause(0.01);
    }

    void show()
    {
        plt::show();
    }

private:
    struct Series
    {
        std::vector<double> x;
        std::vector<double> y;
    };

    int rows_;
    int cols_;

    // subplot -> (series_name -> data)
    std::map<int, std::map<std::string, Series>> data_;
};

#endif