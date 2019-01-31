using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace Agent_Bodies
{
    public class Agent_BodiesInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "AgentBodies";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("a14d3f1b-9b6d-49cb-b712-018ecb6e210f");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}
