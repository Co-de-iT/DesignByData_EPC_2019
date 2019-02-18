using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace AgentSystemFinal
{
    public class AgentSystemFinalInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "AgentSystemFinal";
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
                return new Guid("165872d3-dfcb-4e8b-91d3-b5371ee694dd");
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
