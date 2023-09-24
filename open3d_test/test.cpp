//Open3D
#include <open3d/Open3D.h>
 
//Eigen
#include <Eigen/Dense>
 
 
void xjGetInformation(const QString &pcPath)
{
	/* read PC */
	auto cloud_ptr = std::make_shared<open3d::geometry::PointCloud>();
	if (!open3d::io::ReadPointCloud(pcPath.toStdString(), *cloud_ptr)) { return; }
 
	//点数
	int pointCount = cloud_ptr->points_.size();
	ui.listWidget->addItem("点数：" + QString::number(pointCount));
 
	//包围盒
	Eigen::Vector3d min_bound = cloud_ptr->GetMinBound();
	double minX = min_bound(0);
	double minY = min_bound(1);
	double minZ = min_bound(2);
	ui.listWidget->addItem("minX = " + QString::number(minX, 'f', 4) + 
            ", minY = " + QString::number(minY, 'f', 4) + ", minZ = " + QString::number(minZ, 'f', 4));
	Eigen::Vector3d max_bound = cloud_ptr->GetMaxBound();
	double maxX = max_bound(0);
	double maxY = max_bound(1);
	double maxZ = max_bound(2);
	ui.listWidget->addItem("maxX = " + QString::number(maxX, 'f', 4) + 
            ", maxY = " + QString::number(maxY, 'f', 4) + ", maxZ = " + QString::number(maxZ, 'f', 4));
 
	//单点信息
	double x = 0, y = 0, z = 0;
	for (int i = 0; i < cloud_ptr->points_.size(); i++)
	{
		const Eigen::Vector3d &point = cloud_ptr->points_[i];
		x = point(0);
		y = point(1);
		z = point(2);
		ui.listWidget->addItem("pID="+ QString::number(i)+ ", x = " + QString::number(x, 'f', 4) + 
                    ", y = " + QString::number(y, 'f', 4) + ", Z = " + QString::number(z, 'f', 4));
	}
 
	/* visualize PC */
	open3d::visualization::DrawGeometries({ cloud_ptr });
 
	/* write PC */
	open3d::geometry::PointCloud outputPC;
	for (int i = 0; i < 360; i++)
	{
		outputPC.points_.push_back(Eigen::Vector3d(cos(i), sin(i), cos(i)*sin(i)));
	}
	if (open3d::io::WritePointCloud("F:/outputPC.pcd", outputPC))
	{
		QMessageBox::information(0, "", "OK");
	}
}