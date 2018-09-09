using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;

namespace main
{
    public enum LogType
    {
        INFO,
        DEBUG,
        WARN,
        ERROR
    }

    public class Log
    {
        public int screen_log_limitation = 1000;

        public List<Log> system_log = new List<Log>();

        public DateTime time = DateTime.Now;
        public string info = "";
        public LogType type = LogType.INFO;

        /// <summary>
        /// Get string representaion of log object
        /// </summary>
        /// <returns></returns>
        public string getString()
        {
            string result = "";
            result = time.ToString() + " : " + "[" + type + "] " + info;
            return result;
        }
        /// <summary>
        /// Save log object to local storage
        /// </summary>
        /// <param name="log"></param>
        /// <param name="reset"></param>
        public void robolandSdLog(Log log, bool reset = false)
        {
            if (reset)
                if (File.Exists(Statics.sd_logpath))
                    File.Delete(Statics.sd_logpath);

            FileStream fs = new FileStream(Statics.sd_logpath, FileMode.Append, FileAccess.Write);
            StreamWriter sw = new StreamWriter(fs);
            sw.WriteLine(log.time.ToString() + " : " + "[" + log.type + "] " + log.info);
            sw.Close();
            fs.Close();
        }
        /// <summary>
        /// Add new log to log list and save the log to local storage
        /// </summary>
        /// <param name="info"></param>
        /// <param name="type"></param>
        /// <param name="reset"></param>
        public void addLog(string info, LogType type, bool reset = false)
        {
            Log log = new Log();
            log.info = info;
            log.time = DateTime.Now;
            log.type = type;

            if (system_log.Count > screen_log_limitation)
                system_log.RemoveAt(0);

            if (reset)
                system_log.Clear();

            system_log.Add(log);
            robolandSdLog(log, reset);
        }
    }
}