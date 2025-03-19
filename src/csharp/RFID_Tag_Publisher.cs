using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Serialization.Formatters.Binary;
using System.Security.Cryptography;
using System.Text;
using System.ComponentModel;
using System.Xml.Linq;
using System.Drawing.Printing;
using System.Text;
using System.Drawing;
using System.Threading.Tasks;
using System.Net.Sockets;
using System.Text.Json;

using static System.Runtime.InteropServices.JavaScript.JSType;
// Calls for the reader SDK so I can control the reader requiers SDK to be installed
using Impinj.OctaneSdk; 
// used for doing the matrix math that is required for GP Models and Likelihood
using MathNet;
using MathNet.Numerics;

using ROS2;
//using rfid_ros_package.Msg;
using std_msgs.msg;
namespace ConsoleApplication
{
    class RFID_Tag_Publisher
    {
        static ImpinjReader reader = new ImpinjReader();
        static bool reader_connected = false;
        static string tag_storage = "tag_data_storage.xlsx";
        static double phase_threshold = 0.2;
        static List<string> tag_id = new List<string>();
        static bool report_being_processed = false;
        static string tag_liklihood_storage = "env_id_tag_data.xlsx";
        static bool tags_filtered = false;
        static bool env_id_timer = false;
        static bool loop = true;
        static bool write_data = false;
        static bool data_written = false;
        static bool data_being_written = false;

        private Node node;
        //private Publisher<RFIDTagMsgs> node_publication;
        private int i = 1;
        private Publisher<std_msgs.msg.Float64MultiArray> node_publication;
        private std_msgs.msg.Float64MultiArray msg = new();

        static class VariableDeclarations
        {
            public static int counter;
            public static List<Tag> systemData = new();
            public static List<Tag> stored_tags = new();
            public static List<double> distance = new();
            public static double PowerMultiplier;      
        }

        private RFID_Tag_Publisher()
        {
            RCLdotnet.Init();
            node = RCLdotnet.CreateNode("rfid_topic_1");
            node_publication = node.CreatePublisher<std_msgs.msg.Float64MultiArray>("rfid_topic"); 
            
            node.CreateTimer(TimeSpan.FromSeconds(1.0), PublishTag);
        }

        private void PublishTag(TimeSpan elapsed)
        {
            // msg.freq1_phase = 1;
            // msg.freq2_phase = 2;
            // msg.ant1_phase = 3;
            // msg.ant2_phase = 4;
            // msg.ant3_phase = 5;
            msg.Data = new List<double> { 1, 2, 3, 4, 5 };

            node_publication.Publish(msg);
        }

        private void Spin() => RCLdotnet.Spin(node);

        static void Main(string[] args)
        {
            RFID_Tag_Publisher rfid_topic_1 = new RFID_Tag_Publisher();
            rfid_topic_1.Spin();
        }
    }
}
