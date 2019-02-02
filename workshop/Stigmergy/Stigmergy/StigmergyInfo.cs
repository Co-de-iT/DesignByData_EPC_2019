using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace Stigmergy
{
    public class StigmergyInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "Stigmergy";
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
                return new Guid("6615d84c-4799-4fb3-b9a2-2fbe2dde2358");
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
