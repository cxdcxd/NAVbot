using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using ProtoBuf;
using NetMQ;

namespace main
{
    public partial class frmMain : Form
    {
        int time_out_robot = 0;
        int time_out_matlab = 0;

        CarCommand main_command = new CarCommand();
        CarRobot main_status = new CarRobot();

        Bitmap a = new Bitmap(512, 512, System.Drawing.Imaging.PixelFormat.Format24bppRgb);
        Bitmap b = new Bitmap(1536, 1536, System.Drawing.Imaging.PixelFormat.Format24bppRgb);

        Graphics ga;
        Graphics gb;

        float[] laser_data;
        byte[] map_data;
        int map_size;

        

        public frmMain()
        {
            InitializeComponent();

            Statics.main_config = new Config();
            Statics.loadXMLConfig();

            Statics.main_network = new Network<CarRobot, CarCommand>(Statics.local_robot_port,Statics.robot_port,Statics.main_config.robot_ip);
            Statics.main_network.updated += Main_network_updated;

            Statics.matlab_network = new Network<CarCommand,CarRobot>(Statics.local_matlab_port, Statics.matlab_port, Statics.main_config.matlab_ip);
            Statics.matlab_network.updated += Matlab_network_updated;

            Statics.main_log = new Log();
            Statics.ref_main = this;

            this.Text = "Main RoboCar Interface V " + Statics.version;

            txt_robot_ip.Text = Statics.main_config.robot_ip;
            txt_matlab_ip.Text = Statics.main_config.matlab_ip;

            ga = Graphics.FromImage(a);
            gb = Graphics.FromImage(b);
        }

        private void Matlab_network_updated()
        {
            //Get from matlab
            time_out_matlab = 5;
            main_command = Statics.matlab_network.robot_status;
            sendCommand();
        }

        private void main_timer_Tick(object sender, EventArgs e)
        {
            if ( time_out_robot != 0)
            time_out_robot--;

            if ( time_out_matlab != 0)
            time_out_matlab--;


            if ( time_out_robot == 0 )
            {
                panel_robot.BackColor = Color.DarkRed;
            }
            else
            {
                panel_robot.BackColor = Color.DarkGreen;
            }

            if (time_out_matlab == 0)
            {
                panel_matlab.BackColor = Color.DarkRed;
            }
            else
            {
                panel_matlab.BackColor = Color.DarkGreen;
            }

           

            sendCommand();
        }


        private void btn_beep_Click(object sender, EventArgs e)
        {
            //Beep
        }

        private void btn_led_Click(object sender, EventArgs e)
        {
            //LED
            if ( btn_led.Text == "LED ( Set ON )")
            {
                btn_led.Text = "LED ( Set OFF )";
                main_command.led = true;
            }
            else
            if (btn_led.Text == "LED ( Set OFF )")
            {
                btn_led.Text = "LED ( Set ON )";
                main_command.led = false;
            }
            sendCommand();
        }

        private void btn_reset_Click(object sender, EventArgs e)
        {
            //RESET
            track_speed_1.Value = 128;
            main_command.speed1 = track_speed_1.Value;
            txt_set_speed_1.Text = track_speed_1.Value.ToString();
            sendCommand();
        }

        private void btn_set_target_Click(object sender, EventArgs e)
        {
            //Target
            if ( txt_x.Text != "" && txt_y.Text != "" && txt_theta.Text != "")
            {
                main_command.target_location = new RVector3(float.Parse(txt_x.Text), float.Parse(txt_y.Text), float.Parse(txt_theta.Text));
                sendCommand();
            }


        }

        private void btn_navigation_Click(object sender, EventArgs e)
        {
            //Navigation
        }

        private void btn_cancel_Click(object sender, EventArgs e)
        {
            //Cancel
        }

        private void button1_Click(object sender, EventArgs e)
        {
           //Proto
           string x = Serializer.GetProto<CarCommand>();
           string y = Serializer.GetProto<CarRobot>();
        }

        private void btn_config_Click(object sender, EventArgs e)
        {
            if ( txt_matlab_ip.Text != "" && txt_robot_ip.Text != "")
            {
                Statics.main_config.robot_ip = txt_robot_ip.Text;
                Statics.main_config.matlab_ip = txt_matlab_ip.Text;
                Statics.saveXMLConfig();
            }
        }

        private void Main_network_updated()
        {
            time_out_robot = 5;
            if (Statics.main_network.robot_status.version == 100) return;

            main_status = Statics.main_network.robot_status;

            //Send to matlab
           

            this.Invoke(new MethodInvoker(() =>{

                txt_load.Text = main_status.load.ToString();
                txt_position.Text = main_status.position.ToString();
                txt_speed1.Text = main_status.speed1.ToString();
                txt_speed2.Text = main_status.speed2.ToString();
                txt_alarm.Text = main_status.alarm.ToString();
                txt_sensor.Text = main_status.sensor.ToString();
                txt_voltage.Text = main_status.battery.ToString();

                lst_points.Items.Clear();

                if (main_status.path != null)
                {
                    for (int i = 0; i < main_status.path.Length; i++)
                    {
                        lst_points.Items.Add(main_status.path[i].x + "," + main_status.path[i].y + "," + main_status.path[i].theta);
                    }
                }

                if ( main_status.location != null )
                txt_current_position.Text = main_status.location.x.ToString("N2") + "," + main_status.location.y.ToString("N2") + "," + main_status.location.theta.ToString("N2");

                if ( main_status.target_location != null)
                txt_current_goal_position.Text = main_status.target_location.x.ToString("N2") + "," + main_status.temp_target_location.y.ToString("N2") + "," + main_status.temp_target_location.theta.ToString("N2");

                if ( main_status.laser != null)
                laser_data = main_status.laser.ranges;

                if ( main_status.map_data != null)
                map_data = main_status.map_data;

                map_size = main_status.map_size;
            }));
           
        }

        private void track_speed_1_Scroll(object sender, EventArgs e)
        {
            txt_set_speed_1.Text = track_speed_1.Value.ToString();
            main_command.speed1 = track_speed_1.Value;
            sendCommand();
        }

        private void track_position_Scroll(object sender, EventArgs e)
        {
            txt_set_position.Text = track_position.Value.ToString();
            main_command.position = track_position.Value;
            sendCommand();
        }

        private void frmMain_Load_3(object sender, EventArgs e)
        {
           
        }

        private void btn_reset2_Click(object sender, EventArgs e)
        {
            track_speed_2.Value = 128;
            main_command.speed2 = track_speed_2.Value;
            txt_set_speed_2.Text = track_speed_2.Value.ToString();
            sendCommand();
        }

        private void btn_reset3_Click(object sender, EventArgs e)
        {
            track_position.Value = 128;
            main_command.position = track_position.Value;
            txt_set_position.Text = track_position.Value.ToString();
            sendCommand();
        }

        private void btn_stop_Click(object sender, EventArgs e)
        {
            track_speed_1.Value = 128;
            track_speed_2.Value = 128;
            track_position.Value = 128;

            main_command.speed1 = track_speed_1.Value;
            main_command.speed2 = track_speed_2.Value;
            main_command.position = track_position.Value;

            txt_set_speed_1.Text = track_speed_1.Value.ToString();
            txt_set_speed_2.Text = track_speed_2.Value.ToString();
            txt_set_position.Text = track_position.Value.ToString();

        

            sendCommand();
        }

        private void track_speed_2_Scroll(object sender, EventArgs e)
        {
            txt_set_speed_2.Text = track_speed_2.Value.ToString();
            main_command.speed2 = track_speed_2.Value;
            sendCommand();
        }

        public void sendCommand()
        {
            Statics.main_network.sendMessage(main_command);
        }

        private void frmMain_Load(object sender, EventArgs e)
        {

        }

        private void frmMain_Load_1(object sender, EventArgs e)
        {

        }

        private void frmMain_FormClosed(object sender, FormClosedEventArgs e)
        {
            if ( Statics.main_network != null)
            Statics.main_network.kill();

            if ( Statics.matlab_network != null)
            Statics.matlab_network.kill();
        }

        private void frmMain_Load_2(object sender, EventArgs e)
        {

        }

        

        private void map_timer_Tick(object sender, EventArgs e)
        {
            try
            {
                if (rad_laser.Checked && laser_data != null)
                {
                    float[] laser_data_local = (float[])laser_data.Clone();
                    ga.Clear(Color.White);

                    for (int i = 0; i < laser_data_local.Length; i++)
                    {
                        if (i % 2 == 0)
                        {
                            float value = laser_data_local[i];
                            if (float.IsInfinity(value)) value = 0;

                            int xx = (int)((value / 5 * 512f / 2) * Math.Sin(i * 0.0174533f)) + (int)(512f / 2);
                            int yy = (int)(value / 5 * 512f / 2 * Math.Cos(i * 0.0174533f)) + (int)(512f / 2);

                            ga.DrawEllipse(new Pen(Color.Red), new Rectangle(xx, yy, 3, 3));
                        }
                    }

                    img_map.Image = a;
                }
                //else if ( rad_map.Checked && map_data != null && false)
                //{
                //    byte[] map_data_local = (byte[])map_data.Clone();
                //    gb.Clear(Color.White);

                //    int real_map_size = 1536;
        

                //    for (int i = 0; i < map_data_local.Length; i++)
                //    {
                //        if (i % 10 == 0)
                //        {

                //            int value = map_data_local[i];

                //            int x = i % real_map_size;
                //            int y = i / real_map_size;

                //            if ( value == 0 )
                //            {
                //                gb.DrawEllipse(new Pen(Color.Black), new Rectangle(x, y, 1, 1));
                //            }
                //            else if ( value == -1 )
                //            {
                //                gb.DrawEllipse(new Pen(Color.Blue), new Rectangle(x, y, 1, 1));
                //            }
                           

                            
                //        }
                //    }

                //    img_map.Image = b;
                //}
                
            }
            catch (Exception ee)
            {

            }
        }

        private void frmMain_Load_4(object sender, EventArgs e)
        {

        }

        private void rad_map_CheckedChanged(object sender, EventArgs e)
        {

        }

        private void frmMain_Load_5(object sender, EventArgs e)
        {

        }

        private void frmMain_Load_6(object sender, EventArgs e)
        {

        }

        private void frmMain_Load_7(object sender, EventArgs e)
        {

        }

        private void frmMain_Load_8(object sender, EventArgs e)
        {
            
        }

        private void matlab_Tick(object sender, EventArgs e)
        {
            if (Statics.matlab_network != null && main_status != null)
                Statics.matlab_network.sendMessage(main_status);
        }
    }
}
