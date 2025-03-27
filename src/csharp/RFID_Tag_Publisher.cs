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
// Ros 2 namepsace for C#
using ROS2;
//using rfid_ros_package.Msg;

// package for standard messages for ROS2
using std_msgs.msg;

// Allows for using excel
using OfficeOpenXml;


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
        static bool loop = true;
        static bool write_data = false;
        static bool data_written = false;
        static bool data_being_written = false;
        // string of valid tags that are going to be deployed in the system
        static string[] valid_tag_ids = {"2022 0117 1212 1A06 1A10 0074", 
                                         "2022 0704 1597 9A02 1A10 00C9",
                                         "2021 1019 1212 1A06 1030 0033",
                                         "2021 1019 1212 1A06 1030 004B",
                                         "2021 1019 1212 1A06 1030 0049",
                                         "2021 1019 1212 1A06 1030 004F",
                                         "2021 1019 1212 1A06 1030 0046",
                                         "2021 1019 1212 1A06 1030 0034",
                                         "2021 1019 1212 1A06 1030 0048",
                                         "2021 1019 1212 1A06 1030 004C",
                                         "2021 1019 1212 1A06 1030 0042",
                                         "2021 1019 1212 1A06 1030 0044",
                                         "2021 1019 1212 1A06 1030 003C",
                                         "2021 1019 1212 1A06 1030 003E",
                                         "2021 1019 1212 1A06 1030 004E",
                                         "2021 1019 1212 1A06 1030 0045",
                                         "2021 1019 1212 1A06 1030 003F",
                                         "2021 1019 1212 1A06 1030 003D",
                                         "2021 1019 1212 1A06 1030 003A",
                                         "2021 1019 1212 1A06 1030 003B",
                                         "2021 1019 1212 1A06 1030 0043",
                                         "2021 1019 1212 1A06 1030 004D",
                                         "2021 1019 1212 1A06 1030 0047",
                                         "2021 1019 1212 1A06 1030 0041",
                                         "2021 1019 1212 1A06 1030 0032",
                                         "2021 1019 1212 1A06 1030 0050",
                                         "2021 1019 1212 1A06 1030 0040",
                                         "2021 1019 1212 1A06 1030 0038",
                                         "2021 1019 1212 1A06 1030 0039"};
        static bool tag_report_processed = true;

        // creates the ros node
        private Node node;
        private int i = 1;
        // creates the publisher array        
        // creates a list of publishers that can be used to represent the number of deployed tags
        public static List<Publisher<std_msgs.msg.Float64MultiArray>> publisher_series = new List<Publisher<std_msgs.msg.Float64MultiArray>>();
        public static Publisher<std_msgs.msg.Bool> determine_env = new Publisher<std_msgs.msg.Bool>();
        // number of tags that are deployed in the environment
        static int number_of_tags = valid_tag_ids.Length;
        static int number_of_msgs_in_topic = 5;
        // matrix to hold the information that will need to be published. once all the values have been set, the system will will send a command to
        // fill in the data for that tag topic and publish it. Values are initialized to 0 by default
        static double[,] tag_information_storage = new double[number_of_msgs_in_topic,number_of_tags];
        // these booleans will be checked and if its true then the specific tag topic will be published. The booleans are initialized to false 
        // by C# default        
        static bool[] tag_publish = new bool[number_of_tags]; 
        // creates a message array of floats
        public static std_msgs.msg.Float64MultiArray msg = new();
        public static std_msgs.msg.Bool env_id_msg = new();

        static class VariableDeclarations
        {
            public static int counter;
            // stores tags that were collected during normal operation time
            public static List<Tag> systemData = new();
            // stores tags taht were collected during environmental selection period
            public static List<Tag> stored_tags = new();
        }

        // initiallizes the ros connectiong and creates the nodes
        private RFID_Tag_Publisher()
        {
            RCLdotnet.Init();
            string node_name = "rfid_tag";
            string topicName;
            node = RCLdotnet.CreateNode(node_name);
            for (int i = 0; i < number_of_tags; i++)
            {
                topicName = $"rfid_topic_{i}";
                
                publisher_series.Add(node.CreatePublisher<std_msgs.msg.Float64MultiArray>(topicName));

            }

            determine_env =  node.CreatePublisher<std_msgs.msg.Bool>("env_check");
            
        }

    

        // keeps node active to confinuously obtain data
        private void Spin() => RCLdotnet.Spin(node);

        static void Main(string[] args)
        {
            RFID_Tag_Publisher rfid_topic_1 = new RFID_Tag_Publisher();
            try
            {
                // connects to the reader
                Console.WriteLine("Connecting to reader");       
                reader.Connect("169.254.99.252");
                reader_connected = true;
                
            }
            catch (OctaneSdkException e2)
            {
                try
                {
                    reader.Connect("169.254.1.1");
                    reader_connected = true;

                }
                catch(OctaneSdkException e1)
                {
                    Console.WriteLine(e1.Message);
                }
                
            }

            if(reader_connected)
            { 
                Console.WriteLine("connection established");
                Settings settings = reader.QueryDefaultSettings();
                // tells me which antenna is detecting the tag
                settings.Report.IncludeAntennaPortNumber = true;
                // provides me the phase angle of recieved signal
                settings.Report.IncludePhaseAngle = true;
                // tells me the frequency the tag is being detected at
                settings.Report.IncludeChannel = true;
                // includes time stamp
                settings.Report.IncludeFirstSeenTime = true;
                // Reports each time a tag is detected and the values associated with the tag
                settings.Report.Mode = ReportMode.Individual;
                // Applys the settings to the reader
                reader.ApplySettings(settings);   
                // Collects tags readings and stores the data into list
                reader.Start();
                Console.WriteLine("Reader STarted");
                // creates a stop watch to collect data for environmal identification. It does not need to operate the entire time so its only running for 4 minutes
                Stopwatch stop_watch = new Stopwatch();
                // stop watch used to turn the system off after set of time. It is being used current as a time for turning the system off.
                Stopwatch stop_watch2 = new Stopwatch();
                stop_watch.Start();
                stop_watch2.Start();
                Console.WriteLine(tag_information_storage.GetLength(0));
                Console.WriteLine(tag_information_storage.GetLength(1));
                while(loop)
                {   
                    
                    if(stop_watch.Elapsed.TotalMinutes<=0.5)
                    {
                        //Console.WriteLine("In function 1");  
                        Console.WriteLine(stop_watch.Elapsed.TotalMinutes);
                            // collects the tag data to be used for environmental identification
                        reader.TagsReported += Data_Collection_For_Env_ID;
                    }
                    else
                    {   // switches to ordinary tag filtering for localization 
                        // checks to see if the data has been writen to an excel file yet
                        // will not start looking for tags until the data for environmental identification has been collected

                        if(!data_written)
                        {
                            Console.WriteLine("Writing Data to excel file");
                            data_written = true;
                            // checks to see if the data writing process has started
                            // it being true prevents the function from being called multiple times
                            if(!data_being_written)
                            {
                                data_being_written = true;
                                Write_Data_For_Env_ID();
                            }   
                        }
                        else
                        {
                            env_id_msg.Data = new Bool;
                            determine_env.Publish(env_id_msg);
                            
                            // creates the report and starts filling out the tag information into the matrix
                            if(tag_report_processed)
                            {
                                tag_report_processed = false;
                                reader.TagsReported += OnTagsReported;

                            }
                            
                            
                        }
                        
                    }

                    
                    
                    if(stop_watch.Elapsed.TotalMinutes>10)
                    {
                        reader.Stop(); 
                        reader.Disconnect(); 
                        Console.WriteLine("Disconnected");
                        loop = false;
                    }    
                    
                    System.Threading.Thread.Sleep(500);
                }  

            }

                rfid_topic_1.Spin();
        }

        // The data calculations to determine the environment based off the radio signal is done
        // on python due to the required computations. Therefore, all the data for the entire monitoring period
        // is copied over to excel to be processed later
        static void Write_Data_For_Env_ID()
        {
            data_being_written = true;
            Console.WriteLine("writing env data");
            try
            {
                string_file_path = "detected_data.txt";
                using (StreamWriter writer = new StreamWriter(filePath))
                {
                    foreach (Tag tag in VariableDeclarations.stored_tags)
                    {
                        string id = tag.Epc.ToString();
                        string phase = tag.PhaseAngleInRadians.ToString();
                        string freq = tag.ChannelInMhz.ToString();
                        writer.WriteLine($"{id}\t{phase}\t{channel}");  
                    }
                    // Write header
                }
                
                data_written = true;
            }
            catch(Exception ex)
            {
                Console.WriteLine(ex.Message);
            }
            
            // for use later                  

        }

        // Collects the tag data that will need to be used for environmental identification. Once the environment has been identified, this function will not be used again
        // but it will momentarily be called every few minutes in order to update the reader to ensure optimal localization is available
        static void Data_Collection_For_Env_ID(ImpinjReader sender, TagReport report)
        {
            
            tags_filtered = true;
            string tag_string;
            int tag_location;
            foreach (Tag tag in report)
            {                
                // converts the tag data object to a string for comparison
                tag_string = tag.Epc.ToString();
                // currently the tags being used start with 2022 but this will be changed to a database of tags that are being measured. this will be replaced with that database once it is
                // created for now its just this
                tag_location = Array.IndexOf(valid_tag_ids, tag_string);


                if(tag_location != -1)
                {
                    // makes sures the tag data is only recorded when its in the operational frequency
                    if((tag.ChannelInMhz == 902.75) || (tag.ChannelInMhz == 903.75))
                    {
                        // eventually we will also limit it to one antenna to simplify calculations but that will have to wait for which antenna.
                        // chose the second antenna as it is on the one in middle
                        if(tag.AntennaPortNumber == 2)
                        {
                            VariableDeclarations.stored_tags.Add(tag);
                        }
                        
                    }
                    
                }
            }
            tags_filtered = false;

        }

            /* This function is being used when the environment has already been identified and the system is ready to start tracking the tags in real time. At this point
            the system will be in full complete contact with the python script and will now be tracking all defined assets
            sender and report are specific calls that are required to report tags for the reader. they are not messed with on our end and are handeled entirely by the octane sdk.
            instead of all of the data that is being collected to be stored, the data will only be storing the latest collection of tags that are inside of the tag report.
            */
            static void OnTagsReported(ImpinjReader sender, TagReport report)
            {
                
                report_being_processed = true;

                bool update_tag;
                string tag_string;
                double previous_phase = 0;
                int phase_storage_index = 0;
                double incoming_phase = 0;
                double new_phase;
                int tag_location;
                     
                try
                {
                    // Each report only has one tag. So I can go through here get the information then publish the topic
                    foreach (Tag tag in report)
                    {                           
                        // adds the tag to the tag report at the end as a record. Dont really have a reason why it does it but its easy to get rid of
                       // VariableDeclarations.systemData.Add(tag);  
                        // ensures only our tags are being saved and stored.                         
                        tag_string = tag.Epc.ToString();
                        // checks that the tag we detected is actually in the array of viable tags. 
                        tag_location = Array.IndexOf(valid_tag_ids, tag_string);
                        
                        if(tag_location != -1)
                        {                      
                            
                            // ensures only data that is collected is in the frequency the system
                            // is calibrated for
                            if((tag.ChannelInMhz == 902.75) || (tag.ChannelInMhz == 903.75))
                            {
                                // outputs to console as a type of debugging
                                
                                // deals with filling out the elements on the first two phases
                                // only the centeral antenna will provide phases to be used for ranging. 
                                // Since all antennas are so close together. We cannot do triangulation
                                Console.WriteLine(tag_string);
                                Console.WriteLine(tag_location);
                                Console.WriteLine(tag.ChannelInMhz);
                                Console.WriteLine(tag.PhaseAngleInRadians);
                                Console.WriteLine(tag.AntennaPortNumber);
                                if(tag.AntennaPortNumber == 2)
                                {
                                    if(tag.ChannelInMhz == 902.75)
                                    {
                                        if(tag_information_storage[0,tag_location] == 0)
                                        {
                                            tag_information_storage[0,tag_location] = tag.PhaseAngleInRadians;
                                        }
                                        else
                                        {
                                            tag_information_storage[0,tag_location] = Phase_Calculation(tag_information_storage[0,tag_location],tag.PhaseAngleInRadians);
                                        }
                                    }
                                    else if(tag.ChannelInMhz == 903.75)
                                    {
                                        if(tag_information_storage[1,tag_location] == 0)
                                        {
                                            tag_information_storage[1,tag_location] = tag.PhaseAngleInRadians;
                                        }
                                        else
                                        {
                                            tag_information_storage[1,tag_location] = Phase_Calculation(tag_information_storage[1,tag_location],tag.PhaseAngleInRadians);
                                        }
                                    }                                    
                                    
                                }
                            
                            }

                            // updates the antenna information. due to tag read rate, it should be reading 100-1000 tags per second so we just get this 
                            // information for the topic before the person even really had time to move (hopefully)
                            if(tag.ChannelInMhz == 902.75)
                            {
                                if(tag_information_storage[tag.AntennaPortNumber+1,tag_location] == 0)
                                {
                                    tag_information_storage[tag.AntennaPortNumber+1,tag_location] = tag.PhaseAngleInRadians;
                                }
                                else
                                {
                                    tag_information_storage[tag.AntennaPortNumber+1,tag_location] = Phase_Calculation(tag_information_storage[tag.AntennaPortNumber+1,tag_location],tag.PhaseAngleInRadians);
                                }        
                            }  


                        }              
                        
                    }
                    
                    
                }
                catch(Exception ex)
                {
                    Console.WriteLine(ex.Message);
                }      

                bool condition;
                int b;
                double f1_phase;
                double f2_phase;
                double a1_phase;
                //double a2_phase;
                //double a3_phase;
               
                for(int a = 0; a < number_of_tags; a++)
                {
                    Console.WriteLine("Checking Topic");
                    Console.WriteLine(tag_information_storage[0,a]);
                    Console.WriteLine(tag_information_storage[1,a]);
                    Console.WriteLine(tag_information_storage[2,a]);
                    b = 0;
                    condition = true;
                    // checks each  value in the array. it stops either when the entire array column is checked or it hits a false value. 
                
                    while(condition && b < number_of_msgs_in_topic)
                    {
                        condition = tag_information_storage[b,a] != 0;
                        b++;
                    }

                    if(condition)
                    {
                        // msg.freq1_phase;
                        // msg.freq2_phase;
                        // msg.ant1_phase;
                        // msg.ant2_phase;
                        // msg.ant3_phase;
                        Console.WriteLine("Publishing");
                        f1_phase = tag_information_storage[0,a];
                        f2_phase = tag_information_storage[1,a];
                        a1_phase = tag_information_storage[2,a];
                        Console.WriteLine(f1_phase);
                        Console.WriteLine(f2_phase);
                        Console.WriteLine(a1_phase);
                        //a2_phase = tag_information_storage[a,3];
                        //a3_phase = tag_information_storage[a,4];
                        msg.Data = new List<double> { f1_phase, f2_phase, a1_phase,a};

                        publisher_series[a].Publish(msg);

                    } 
                    
                    
                }
                tag_report_processed = true;


            }

            // calculates the new phase. RFID Impinj readers have alot of phase wrapping between - 2pi and 2pi. this function undoes the phase wrapping 
            // by comparing the phases to previous calculated phases. 

            static double Phase_Calculation(double previous_phase,double incoming_phase)
            {
                double new_phase = 0;

                if(Math.Abs(Math.PI + incoming_phase - previous_phase) <=phase_threshold)
                {
                    new_phase = Math.Abs(incoming_phase + Math.PI);
                }
                else if(Math.Abs(Math.Abs(Math.PI - incoming_phase) - previous_phase) <=phase_threshold)
                {
                    new_phase = Math.Abs(Math.PI - incoming_phase);
                }
                else if(Math.Abs(2*Math.PI - incoming_phase - previous_phase)<= phase_threshold)
                {
                    new_phase = 2*Math.PI - incoming_phase;
                }    

                return new_phase;

            }

            // C# doesnt have a built in fnction to check for non zero values so its got to be checked value by value
            


    }


}
