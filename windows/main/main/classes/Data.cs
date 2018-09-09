using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace main
{
    [Serializable]
    [ProtoContract]
    public class RVector3
    {
        [ProtoMember(1)]
        public float x = 0;
        [ProtoMember(2)]
        public float y = 0;
        [ProtoMember(3)]
        public float theta = 0;

        public RVector3(float a, float b, float c)
        {
            x = a;
            y = b;
            theta = c;
        }

        public RVector3()
        {

        }
    }

    [Serializable]
    [ProtoContract]
    public class Laser
    {
        [ProtoMember(1)]
        public float[] ranges; //m

        [ProtoMember(2)]
        public int robot_id; //the robot id

        [ProtoMember(3)]
        public float angel_min; //rad

        [ProtoMember(4)]
        public float angel_max; //rad

        [ProtoMember(5)]
        public float angel_increment; //rad

        [ProtoMember(6)]
        public float range_min; //m

        [ProtoMember(7)]
        public float range_max; //m

        [ProtoMember(8)]
        public float time_increment; //sec

        [ProtoMember(9)]
        public float scan_time; //sec

        public Laser()
        {

        }
    }

    [Serializable]
    [ProtoContract]
    public class CarRobot
    {
        [ProtoMember(1)]
        public int id;
        [ProtoMember(2)]
        public RVector3 location;
        [ProtoMember(3)]
        public RVector3 target_location;
        [ProtoMember(4)]
        public RVector3 temp_target_location;
        [ProtoMember(5)]
        public int state;
        [ProtoMember(6)]
        public float battery;
        [ProtoMember(7)]
        public RVector3[] path;
        [ProtoMember(8)]
        public RVector3[] temp_path;
        [ProtoMember(9)]
        public int temprature = 0;
        [ProtoMember(10)]
        public Laser laser;
        [ProtoMember(11)]
        public int version;
        [ProtoMember(12)]
        public ulong time_span;
        [ProtoMember(13)]
        public int speed1;
        [ProtoMember(14)]
        public int speed2;
        [ProtoMember(15)]
        public int position;
        [ProtoMember(16)]
        public int load;
        [ProtoMember(17)]
        public int sensor;
        [ProtoMember(18)]
        public int alarm;
        [ProtoMember(19)]
        public byte[] map_data;
        [ProtoMember(20)]
        public int map_size;

        public CarRobot()
        {
        }
    }

    [Serializable]
    [ProtoContract]
    public class CarCommand
    {
        [ProtoMember(1)]
        public string cmd = "";
        [ProtoMember(2)]
        public RVector3 target_location = null;
        [ProtoMember(3)]
        public RVector3 temp_target_location = null;
        [ProtoMember(4)]
        public int version = 1;
        [ProtoMember(5)]
        public ulong time_span = 0;
        [ProtoMember(6)]
        public int speed1 = 128;
        [ProtoMember(7)]
        public int speed2 = 128;
        [ProtoMember(8)]
        public int position = 128;
        [ProtoMember(9)]
        public bool beep = false;
        [ProtoMember(10)]
        public bool led = false;


        public CarCommand()
        {
        }
    }
   
}
