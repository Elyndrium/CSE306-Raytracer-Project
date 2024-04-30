#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <limits>
#include <thread>
#include <iostream>
#include <stdio.h>
#include <string>
#include <array>
#include <vector>

typedef std::vector<std::vector<std::array<unsigned char, 3>>> ImageVec;

bool parse_int(int &value, char arg[]){
    try {
        value = std::stoi(arg);
        return true;
    } catch (...) {
        std::cerr << "Error occured parsing arguments" << std::endl;
        return false;
    }
}

void to_array(ImageVec& from, unsigned char* to){
    for (size_t i=0; i<from.size(); ++i){
        for (size_t j=0; j<from[0].size(); ++j){
            for (int k=0; k<3; ++k){
                to[((i*from[0].size() + j) * 3) + k] = from[i][j][k];
            }
        }
    }
}

void to_vec(unsigned char* from, ImageVec& to, int W, int H, int nChannels = 3){
    to.resize(H, std::vector<std::array<unsigned char, 3>>(W));
    for (int i=0; i<H; ++i){
        for (int j=0; j<W; ++j){
            for (int k=0; k<3; ++k){
                to[i][j][k] = from[((i*W + j) * nChannels) + k];
            }
        }
    }
}

void channel_to_bw (std::vector<std::vector<unsigned int>> channel, ImageVec& image){
    // assume image of good size
    unsigned int max = 0;
    for (size_t i=0;i<channel.size();++i){
        for (size_t j=0;j<channel[0].size();++j){
            max = std::max(max, channel[i][j]);
        }
    }
    for (size_t i=0;i<channel.size();++i){
        for (size_t j=0;j<channel[0].size();++j){
            for (int k=0; k<3; ++k){
                image[i][j][k] = (255 * channel[i][j])/max;
            }
        }
    }
}

unsigned char intensity(unsigned char* image, int W, int H, int nChannels, int x, int y){
    if (0<=x && x<W && 0<=y && y<H){return image[(y * W + x) * nChannels + 0]+image[(y * W + x) * nChannels + 1]+image[(y * W + x) * nChannels + 2];}
    std::cout << "Value out of bounds (in double intensity)" << std::endl;
    throw "Value out of bounds (in double intensity)";
}

std::vector<std::vector<unsigned int>> energy_map(unsigned char* image, int W, int H, int nChannels){
    std::vector<std::vector<unsigned int>> energy_image;
    for (int i = 0; i < H; ++i){
        energy_image.push_back(std::vector<unsigned int>(W));
        for (int j = 0; j < W; ++j) {
            unsigned char energy = 0;
            if (j>0 && j<W-1){energy += abs(intensity(image, W, H, nChannels, j+1, i) - intensity(image, W, H, nChannels, j-1, i));}
            if (i>0 && i<H-1){energy += abs(intensity(image, W, H, nChannels, j, i+1) - intensity(image, W, H, nChannels, j, i-1));}
            energy_image[i][j] = energy;
        }
    }
    return energy_image;
}

enum class Direction {hor=0, ver=1};
void accumulate(std::vector<std::vector<unsigned int>>& emap, Direction dir){
    if (dir == Direction::ver){
        for (size_t y = 1; y < emap.size(); ++y){
            for (size_t x = 0; x < emap[0].size(); ++x){
                unsigned int mini = emap[y-1][x];
                if (x>0){mini = std::min(mini, emap[y-1][x-1]);}
                if (x<emap[0].size()-1){mini = std::min(mini, emap[y-1][x+1]);}
                emap[y][x] += mini;
            }
        }
    }
    else{
        for (size_t x = 1; x < emap[0].size(); ++x){
            for (size_t y = 0; y < emap.size(); ++y){
                unsigned int mini = emap[y][x-1];
                if (y>0){mini = std::min(mini, emap[y-1][x-1]);}
                if (y<emap.size()-1){mini = std::min(mini, emap[y+1][x-1]);}
                emap[y][x] += mini;
            }
        }
    }
}

void min_index(std::vector<std::vector<unsigned int>>& emap, Direction dir, size_t coords[2]){
    size_t miny = 0;
    size_t minx = 0;
    unsigned int mini = std::numeric_limits<unsigned int>::max();
    if (dir == Direction::ver){
        miny = emap.size()-1;
        for (size_t x = 0; x < emap[0].size(); ++x){
            if (emap[miny][x] < mini){
                mini = emap[miny][x];
                minx = x;
            }
        }
    } else{
        minx = emap[0].size()-1;
        for (size_t y = 0; y < emap.size(); ++y){
            if (emap[y][minx] < mini){
                mini = emap[y][minx];
                miny = y;
            }
        }
    }
    if (miny == 0 && minx == 0){std::cout << "min_index bug or image too small" << std::endl;}
    coords[0] = miny;
    coords[1] = minx;
}

void remove_line(std::vector<std::vector<unsigned int>>& emap, Direction dir, ImageVec& image){
    size_t coord[2];
    min_index(emap, dir, coord); // miny, minx
    if (dir == Direction::ver){
        size_t y=coord[0]+1;
        do {
            --y;
            image[y].erase(image[y].begin() + coord[1]);
            size_t next_x = coord[1];
            if (coord[1] > 0 && emap[y][coord[1]-1]<emap[y][next_x]){next_x = coord[1]-1;}
            if (coord[1] < emap[0].size()-1 && emap[y][coord[1]+1]<emap[y][next_x]){next_x = coord[1]+1;}
            coord[1] = next_x;
        } while (y>0);
    } else{
        size_t x=coord[1]+1;
        do {
            x--;
            // be smarter boi | image[coord[0]].erase(image[coord[0]].begin() + x);
            // We move up everything under image[coord[0]][x]
            for (size_t y_move = coord[0]; y_move<image.size()-1; ++y_move){
                image[y_move][x] = image[y_move+1][x];
            }

            size_t next_y = coord[0];
            if (coord[0] > 0 && emap[coord[0]-1][x]<emap[next_y][x]){next_y = coord[0]-1;}
            if (coord[0] < emap.size()-1 && emap[coord[0]+1][x]<emap[next_y][x]){next_y = coord[0]+1;}
            coord[0] = next_y;
        } while (x>0);
        image.erase(image.end() - 1);
        
    }
}

int main(int argc, char* argv[]){
    if (argc != 5){
        std::cout << "Arguments should be '<str image_path>, <stf new_image_path>, <int new_width>, <int new_heigth>'"<< std::endl;
        return 1;
    }
    int newWidth, newHeigth;
    char* image_path = argv[1];
    char* new_image_path = argv[2];
    if (!(parse_int(newWidth, argv[3]) && parse_int(newHeigth, argv[4]))){
            std::cout << "Error parsing 'int' arguments from '<str image_path>, <stf new_image_path>, <int new_width>, <int new_heigth>'" << std::endl;
            return 1;
    }
    int W, H, nChannels;
    unsigned char* image;

    if (stbi_info(image_path, &W, &H, &nChannels) == 0){
            throw "Error loading UV file";
        }

    // We start the actual computations
    image = stbi_load(image_path, &W, &H, &nChannels, 0);

    ImageVec new_image;
    to_vec(image, new_image, W, H, nChannels);
    // new_image is now the original image in vector form

    size_t remove_verticals = W-newWidth;
    size_t remove_horizontals = H-newHeigth;
    // Start looping over the removal
    double initial_ratio = 1;
    if (remove_horizontals>0){initial_ratio = (double)remove_verticals/(double)remove_horizontals;}

    while (remove_verticals>0 || remove_horizontals>0){
        double ratio = 1;
        Direction dir;
        if (remove_horizontals>0){
            ratio = (double)remove_verticals/(double)remove_horizontals;
            if (ratio>=initial_ratio){dir = Direction::ver;}
            else {dir = Direction::hor;}
        } else{
            dir = Direction::ver;
        }

        std::vector<std::vector<unsigned int>> emap = energy_map(image, W, H, nChannels);

        accumulate(emap, dir); // ver is when we REMOVE verticals

        remove_line(emap, dir, new_image);

        if (dir == Direction::hor){
            H--;
            remove_horizontals--;
        }
        else {
            W--;
            remove_verticals--;
        }
    }

    unsigned char* array_new_img = new unsigned char[newWidth * newHeigth * 3];
    to_array(new_image, array_new_img);
    stbi_write_png(new_image_path, newWidth, newHeigth, 3, array_new_img, 0);
    delete array_new_img;

    std::cout << "Finished execution" << std::endl;
    return 0;
}
