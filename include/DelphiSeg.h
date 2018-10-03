#define PCL_NO_PRECOMPILE
#include <ros/ros.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <velodyne_pointcloud/point_types.h>

#define VPoint velodyne_pointcloud::PointXYZIR


class DelphiSeg
{
public:

	pcl::PointCloud<VPoint>::Ptr cloudData;
	pcl::PointCloud<VPoint>::Ptr nonGroundPoints;
	pcl::PointCloud<VPoint>::Ptr groundPoints;

	DelphiSeg():
	cloudData(new pcl::PointCloud<VPoint>),
	nonGroundPoints(new pcl::PointCloud<VPoint>),
	groundPoints(new pcl::PointCloud<VPoint>)
	{}

	void ReadKitti(char const * fp)
	{
		// allocate 4 MB buffer (only ~130*4*4 KB are needed)
		int32_t num = 1000000;
		float *data = (float*)malloc(num*sizeof(float));

		// pointers
		float *px = data+0;
		float *py = data+1;
		float *pz = data+2;
		float *pr = data+3;

		int ring = 0;

		// load point cloud
		FILE *stream;
		stream = fopen (fp,"rb");
		num = fread(data,sizeof(float),num,stream)/4;
		cloudData->height=1;
		for (int32_t i=0; i<num; i++) {
			VPoint current_pt;
			current_pt.x = *px;
			current_pt.y = *py;
			current_pt.z = *pz;
			current_pt.intensity = *pr;

			if (i > 0 && cloudData->points[i - 1].y < 0 && current_pt.y > 0)
				ring++;
			current_pt.ring = ring;

			cloudData->push_back(current_pt);
			px+=4; py+=4; pz+=4; pr+=4;
		}
		fclose(stream);

	    // Chop off points that are more than 50m away from the sensor
	    pcl::PassThrough<VPoint> pass;
	    pass.setInputCloud(cloudData);
	    pass.setFilterFieldName ("x");
	    pass.setFilterLimits (-50.0, 50.0);
	    pass.filter (*cloudData);
	}

	void groundRemoval(int Nseg, int Niter, int Nlpr, float ThSeeds, float ThDist)
	{
		float segLen = 100.0 / Nseg;
		for (int i = 0; i < Nseg; i++)
		{
			pcl::PassThrough<VPoint> segFilter;
	    	segFilter.setInputCloud(cloudData);
	    	segFilter.setFilterFieldName ("x");
	    	segFilter.setFilterLimits(-50.0 + segLen*i, -50 + segLen*(i+1));
	    	pcl::PointCloud<VPoint>::Ptr currSegment (new pcl::PointCloud<VPoint>);
	    	segFilter.filter(*currSegment);

	    	/**
	    	* extract initial seeds
	    	*/
	    	VPoint maxHeap[Nlpr+1];
	    	for (int j = 1; j < Nlpr; j++)
	    		maxHeap[j] = currSegment->points[j - 1];
	    	// initializing heap
	    	for (int j = Nlpr / 2; j > 0; j--)
	    		for (int k = j; k < Nlpr; k *= 2)
	    		{
	    			int maxChildIdx;
	    			if (k * 2 + 1 > Nlpr)
	    				maxChildIdx = k * 2;
	    			else
	    			{
	    				if (maxHeap[k * 2].z > maxHeap[k * 2 + 1].z)
	    					maxChildIdx = k * 2;
	    				else
	    					maxChildIdx = k * 2 + 1;
	    			}

	    			if (maxHeap[k].z > maxHeap[maxChildIdx].z)
	    				break;

	    			VPoint tmp;
	    			tmp = maxHeap[k];
	    			maxHeap[k] = maxHeap[maxChildIdx];
	    			maxHeap[maxChildIdx] = tmp;
	    		}

	    	for (int j = Nlpr; j < currSegment->size(); j++)
	    		if (currSegment->points[j].z < maxHeap[1].z)
	    		{
	    			maxHeap[1] = currSegment->points[j];
		    		for (int k = j; k < Nlpr; k *= 2)
		    		{
		    			int maxChildIdx;
		    			if (k * 2 + 1 > Nlpr)
		    				maxChildIdx = k * 2;
		    			else
		    			{
		    				if (maxHeap[k * 2].z > maxHeap[k * 2 + 1].z)
		    					maxChildIdx = k * 2;
		    				else
		    					maxChildIdx = k * 2 + 1;
		    			}

		    			if (maxHeap[k].z > maxHeap[maxChildIdx].z)
		    				break;

		    			VPoint tmp;
		    			tmp = maxHeap[k];
		    			maxHeap[k] = maxHeap[maxChildIdx];
		    			maxHeap[maxChildIdx] = tmp;
		    		}
	    		}

	    	int lprHeight = 0;
	    	for (int j = 1; j <= Nlpr; j++)
	    		lprHeight += maxHeap[j].z;
	    	lprHeight /= Nlpr;

	    	pcl::PointCloud<VPoint>::Ptr tmpGroundPoints (new pcl::PointCloud<VPoint>);
	    	pcl::PointCloud<VPoint>::Ptr tmpNonGroundPoints (new pcl::PointCloud<VPoint>);
	    	
	    	for (int j = 0; j < currSegment->size(); j++)
	    		if (currSegment->points[j].z < lprHeight + ThSeeds)
	    			tmpGroundPoints->push_back(currSegment->points[j]);
	    		else
	    			tmpNonGroundPoints->push_back(currSegment->points[j]);

	    	/**
	    	* ground plane estimation iterations
	    	*/
	    	for (int j = 0; j < Niter; j++)
	    	{
	    		Eigen::Vector4f plane_model(0, 0, 0, 0);
	    		float curvature;
	    		pcl::computePointNormal(*tmpGroundPoints, plane_model, curvature);
	    		tmpGroundPoints->clear();
	    		tmpNonGroundPoints->clear();

	    		for (int k = 0; k < currSegment->size(); k++)
	    		{
	    			Eigen::Vector4f currPt(currSegment->points[k].x, currSegment->points[k].y, currSegment->points[k].z, 1);
	    			if (plane_model.transpose() * currPt < ThDist)
	    				tmpGroundPoints->push_back(currSegment->points[k]);
	    			else
	    				tmpNonGroundPoints->push_back(currSegment->points[k]);
	    		}
	    	}
	    	*groundPoints += *tmpGroundPoints;
	    	*nonGroundPoints += *tmpNonGroundPoints;
	    }
	}

private:
	
};