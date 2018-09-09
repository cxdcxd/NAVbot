using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Xml.Serialization;

namespace main
{
    [XmlRootAttribute("config")]
    public class Config
    {
        public string robot_ip = "192.168.1.7";
        public string matlab_ip = "127.0.0.1";
    }

    class Statics
    {
        public static frmMain ref_main;
        public static string version = "1.0.0";
        public static string sd_folder = "C:\\car\\";
        public static string sd_logpath = "C:\\car\\log.txt";
        public static string sd_config_path = "C:\\car\\config.txt";
        public static Network<CarRobot,CarCommand> main_network;
        public static Network<CarCommand,CarRobot> matlab_network;
        public static Log main_log;
        public static Config main_config;

        public static string robot_port = "7888";
        public static string local_robot_port = "7889";
        public static string matlab_port = "8888";
        public static string local_matlab_port = "8889";

        #region save_load
        public static void XMLSaveData(string path, Type type, object obj)
        {
            try
            {
                XmlSerializer serializer = new XmlSerializer(type);
                TextWriter writer = new StreamWriter(path);
                serializer.Serialize(writer, obj);
                writer.Close();
            }
            catch (Exception e)
            {
            }
        }
        public static object XMLLoadData(string path, Type type)
        {
            object output = null;

            XmlSerializer serializer = new XmlSerializer(type);
            FileStream reader = new FileStream(path, FileMode.Open);
            output = serializer.Deserialize(reader);
            reader.Close();

            return output;
        }
        public static void saveXMLConfig()
        {
            Statics.XMLSaveData(sd_config_path, typeof(Config), Statics.main_config);
        }
        public static void loadXMLConfig()
        {
            bool r2 = System.IO.Directory.Exists(sd_folder);
            if (r2 == false)
            {
                System.IO.Directory.CreateDirectory(sd_folder);
            }
            bool r = System.IO.File.Exists(sd_config_path);
            if (r == false) { saveXMLConfig(); return; }

            try
            {
                Statics.main_config = (Config)Statics.XMLLoadData(sd_config_path, typeof(Config));
            }
            catch (Exception e)
            {
                saveXMLConfig();
            }
        }
        #endregion
    }
}
