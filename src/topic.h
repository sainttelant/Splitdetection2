#ifndef __TOPIC_h___
#define __TOPIC_h___

#include <nlohmann/json.hpp>

#include <string>
#include <vector>

using nlohmann::json;
using namespace std;

namespace inferBboxNs
{
    typedef struct
    {
        float left;
        float top;
        float width;
        float height;
    }Rect;

    typedef struct {
        int64_t   timestamp;
        string   camNo;
        vector<Rect> Bbox;
    }InferBbox;

    void to_json(nlohmann::json& j, const Rect& r)
    {
        j = nlohmann::json{
            {"left",r.left},
            {"top",r.top},
            {"width",r.width},
            {"height",r.height}
        };
    }
    void to_json(nlohmann::json& j, const InferBbox& i)
    {
        j = nlohmann::json{
            {"timestamp",i.timestamp},
            {"camNo",i.camNo},
            {"Bbox",i.Bbox}
        };
    }
    void from_json(const nlohmann::json& j, Rect& r)
    {
        j.at("left").get_to(r.left);
        j.at("top").get_to(r.top);
        j.at("width").get_to(r.width);
        j.at("height").get_to(r.height);
    }

    void from_json(const nlohmann::json& j, InferBbox& i)
    {
        j.at("timestamp").get_to(i.timestamp);
        j.at("camNo").get_to(i.camNo);
        j.at("Bbox").get_to(i.Bbox);
    }

}

namespace abdObjRectNs
{
    typedef struct
    {
        float left;
        float top;
        float width;
        float height;
    }Rect;
   // 抛洒物输出：
        typedef struct {
        int64_t   timestamp;
        string   camNo;
        vector<Rect> Bbox;
    }AbdObjRect;

        void to_json(nlohmann::json& j, const Rect& r)
        {
            j = nlohmann::json{
                {"left",r.left},
                {"top",r.top},
                {"width",r.width},
                {"height",r.height}
            };
        }
        void to_json(nlohmann::json& j, const AbdObjRect& a)
        {
            j = nlohmann::json{
                {"timestamp",a.timestamp},
                {"camNo",a.camNo},
                {"Bbox",a.Bbox}
            };
        }

        void from_json(const nlohmann::json& j, Rect& r)
        {
            j.at("left").get_to(r.left);
            j.at("top").get_to(r.top);
            j.at("width").get_to(r.width);
            j.at("height").get_to(r.height);
        }
        void from_json(const nlohmann::json& j, AbdObjRect& i)
        {
            j.at("timestamp").get_to(i.timestamp);
            j.at("camNo").get_to(i.camNo);
            j.at("Bbox").get_to(i.Bbox);
        }
}
#endif

