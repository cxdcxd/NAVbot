using NetMQ;
using NetMQ.Sockets;
using ProtoBuf;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Matlab
{
    public class Network<Status, Command>
    {
        public System.Timers.Timer main_timer = new System.Timers.Timer();
        public bool connected = false;
        public int time_out = 5;
        public delegate void dupdated();
        public event dupdated updated;

        public delegate void dconnected();
        public event dconnected connection;

        public Status robot_status;

        PublisherSocket publisher;
        SubscriberSocket subscriber;

        bool app_exit = false;

        System.Threading.Thread thread;

        bool old_state = false;
        bool first_time = false;

        void updateStatus()
        {
            old_state = connected;
            triggerConnection();
        }
        private void Main_timer_Elapsed(object sender, System.Timers.ElapsedEventArgs e)
        {
            time_out--;
            if (time_out == 0)
            {
                time_out = 5;
                connected = false;
            }

            if (first_time == false)
            {
                first_time = true;
                updateStatus();
            }
            else
            {
                //Handle State Changes
                if (old_state != connected)
                {
                    updateStatus();
                }
            }
        }

        public void trigger()
        {
            if (updated != null) updated();
        }

        public void triggerConnection()
        {
            if (connection != null) connection();
        }

        public Network(string ip,string local_port,string remote_port)
        {
            publisher = new PublisherSocket();
            subscriber = new SubscriberSocket();

            publisher.Bind("tcp://*:" + local_port);
            subscriber.Subscribe("");
            subscriber.Connect("tcp://" + ip + ":" + remote_port);

            thread = new System.Threading.Thread(new System.Threading.ThreadStart(zmqThread));
            thread.Start();

            main_timer = new System.Timers.Timer();
            main_timer.Interval = 1000;
            main_timer.Elapsed += Main_timer_Elapsed;
            main_timer.Start();
            main_timer.Enabled = true;
        }

        public void kill()
        {
            app_exit = true;

            if (thread != null)
            {
                thread.Abort();
                thread = null;
            }

            if (publisher != null)
            {
                publisher.Close();
            }

            if (subscriber != null)
            {
                subscriber.Close();
            }
        }

        public void sendMessage(Command cmd)
        {
            try
            {
                System.IO.MemoryStream ms = new System.IO.MemoryStream();
                Serializer.Serialize<Command>(ms, cmd);
                Msg msg = new Msg();
                msg.InitGC(ms.ToArray(), (int)ms.Length);
                publisher.Send(ref msg, false);
            }
            catch
            {

            }
        }

        void zmqThread()
        {
            while (app_exit == false)
            {
                if (subscriber != null)
                {
                    try
                    {
                        Msg msg = new Msg();
                        msg.InitEmpty();
                        subscriber.Receive(ref msg);

                        System.IO.MemoryStream ms = new System.IO.MemoryStream(msg.Data);
                        robot_status = Serializer.Deserialize<Status>(ms);
                        trigger();
                        System.Threading.Thread.Sleep(1);
                    }
                    catch
                    {

                    }
                }
            }
        }
    }
}
