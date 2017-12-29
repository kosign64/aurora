#include "astar.h"
#include <ros/ros.h>
#include <QDebug>

using namespace std;

static double mapDistance(const MapPoint &p1, const MapPoint &p2);
static bool checkNeighbors(const MapPoint &p, const Map &map);

AStar::AStar(QObject *parent) : QObject(parent),
    start_{-100, -100},
    stop_{-100, -100}
{

}

void AStar::findPath()
{
    if(map_.map.empty()) return;
    for(auto &data : map_.map)
    {
        if(data == 50) data = 0;
    }
    startPath_ = xyToMapPoint(start_);
    stopPath_ = xyToMapPoint(stop_);
    if(map_(stopPath_) /*||
            !checkNeighbors(stopPath_, map_)*/) return;
    data_.clear();
    AStarPoint startPoint(startPath_);
    startPoint.weight = 0;
    startPoint.pathWeight = 0;
    data_.push_back(startPoint);
    int index = -1;
    try
    {
        while(index == -1)
        {
            algorithm();
            index = isDataContains(stopPath_);
        }
    }
    catch(const std::runtime_error &e)
    {
        ROS_WARN_STREAM(e.what());
        return;
    }
    path_.clear();
    for(const auto &p : data_[index].smallestPath)
    {
        map_(p.x, p.y) = 50;
        path_.push_back(mapPointToxy(p));
    }
    sendPath(path_);
}

void AStar::findPath(const Point2D &start, const Point2D &stop)
{
    setStart(start.x, start.y);
    setStop(stop.x, stop.y);
    findPath();
}

void AStar::reduceMap()
{
    int reduceFactor = 2;
    float resolution = map_.resolution * reduceFactor;
    int width = map_.width / reduceFactor;
    int height = map_.height / reduceFactor;
    vector<int8_t> mapData;
    for(int y = 0; y < (map_.height - (reduceFactor - 1)); y +=
        reduceFactor)
    {
        for(int x = 0; x < (map_.width - (reduceFactor - 1)); x +=
            reduceFactor)
        {
            int8_t maxValue = -1;
            bool foundMax = false;
            for(int i = 0; i < reduceFactor && !foundMax; ++i)
            {
                for(int j = 0; j < reduceFactor && !foundMax; ++j)
                {
                    if(map_(x + i, y + j) > maxValue)
                    {
                        maxValue = map_(x + i, y + j);
                        if(maxValue == 100) foundMax = true;
                    }
                }
            }
            mapData.push_back(maxValue);
        }
    }
    map_.map = std::move(mapData);
    map_.width = width;
    map_.height = height;
    map_.resolution = resolution;
}

void AStar::dilateMap()
{
    vector<int8_t> dilatedMap = map_.map;
    for(int x = 1; x < (map_.width - 1); ++x)
    {
        for(int y = 1; y < (map_.height -1); ++y)
        {
            if(!checkNeighbors(MapPoint{x, y}, map_))
            {
                dilatedMap[x + y * map_.width] = 100;
            }
        }
    }
    map_.map = std::move(dilatedMap);
}

MapPoint AStar::xyToMapPoint(Point2D p)
{
    return xyToMapPoint(p.x, p.y);
}

MapPoint AStar::xyToMapPoint(float x, float y)
{
    MapPoint point;
    x -= map_.x;
    y -= map_.y;

    point.x = round(x / map_.resolution);
    point.y = round(y / map_.resolution);

    return point;
}

void AStar::mapPointToxy(MapPoint point, float &x, float &y)
{
    x = point.x * map_.resolution + map_.x;
    y = point.y * map_.resolution + map_.y;
}

Point2D AStar::mapPointToxy(MapPoint point)
{
    Point2D res;
    mapPointToxy(point, res.x, res.y);

    return res;
}

void AStar::algorithm()
{
    int bestIndex = -1;
    double smallestWeight = std::numeric_limits<double>::max();
    for(size_t i = 0; i < data_.size(); ++i)
    {
        const AStarPoint &p = data_[i];
        if(!p.checked && (p.weight < smallestWeight))
        {
            bestIndex = i;
            smallestWeight = p.weight;
        }
    }
    if(bestIndex == -1) throw std::runtime_error("There is no free way"
                                                 " from start point to"
                                                 " end point");
    AStarPoint best = data_[bestIndex];
    const MapPoint points[] =
    {{best.point.x - 1, best.point.y - 1},
     {best.point.x - 1, best.point.y    },
     {best.point.x - 1, best.point.y + 1},
     {best.point.x    , best.point.y + 1},
     {best.point.x + 1, best.point.y + 1},
     {best.point.x + 1, best.point.y    },
     {best.point.x + 1, best.point.y - 1},
     {best.point.x    , best.point.y - 1}};
    for(const auto &point : points)
    {
        checkPoint(point, best);
    }
    data_[bestIndex].checked = true;
}

int AStar::isDataContains(const MapPoint &point) const
{
    for(size_t i = 0; i < data_.size(); ++i)
    {
        const AStarPoint &p = data_[i];
        if((p.point.x == point.x) && (p.point.y == point.y))
        {
            return i;
        }
    }

    return -1;
}

void AStar::checkPoint(const MapPoint &point,
                       const AStar::AStarPoint &prev)
{
    if(map_(point.x, point.y) == 100 ||
            map_(point.x, point.y) == -1) return;
    if(point.x < 1 || point.x >= (map_.width - 1) ||
            point.y < 1 || point.y >= (map_.height - 1)) return;
    //if(!checkNeighbors(point, map_)) return;
    AStarPoint starPoint(point);
    starPoint.pathWeight = prev.pathWeight + mapDistance(point, prev.point);
    starPoint.weight = mapDistance(point, stopPath_) * 1.2 + starPoint.pathWeight;
    int index = isDataContains(point);
    if(index != -1)
    {
        if(data_[index].weight > starPoint.weight)
        {
            starPoint.smallestPath = prev.smallestPath;
            starPoint.smallestPath.push_back(prev.point);
            data_[index] = std::move(starPoint);
        }
        return;
    }
    starPoint.smallestPath = prev.smallestPath;
    starPoint.smallestPath.push_back(prev.point);
    data_.push_back(std::move(starPoint));
}

void AStar::setMap(const Map &map)
{
    map_ = map;
    //reduceMap();
    for(int i = 0; i < 8; ++i)
    {
        dilateMap();
    }
}

void AStar::setStop(float x, float y)
{
    stop_ = Point2D{x, y};
    if(start_.x != -100 && start_.y != -100)
    {
        findPath();
    }
}

void AStar::setOdometry(double x, double y, double angle)
{
    start_ = Point2D{static_cast<float>(x), static_cast<float>(y)};
    if(stop_.x != -100 && stop_.y != -100)
    {
        findPath();
    }
}

static double mapDistance(const MapPoint &p1, const MapPoint &p2)
{
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

static bool checkNeighbors(const MapPoint &p, const Map &map)
{
    const MapPoint points[] =
    {{p.x - 1, p.y - 1},
     {p.x - 1, p.y    },
     {p.x - 1, p.y + 1},
     {p.x    , p.y + 1},
     {p.x + 1, p.y + 1},
     {p.x + 1, p.y    },
     {p.x + 1, p.y - 1},
     {p.x    , p.y - 1}};
    for(const auto &point : points)
    {
        if(map(point) == 100)
        {
            return false;
        }
    }

    return true;
}
