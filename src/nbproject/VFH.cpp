/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


namespace Decisionsystem
{
    public class VFH
    {
        #region 变量定义
        static int Length = 300;//栅格图长度
        static int Width = 100;//栅格图宽度

        static int VehicleLength = 20;//车身长度

        public double ThresholdValue = 205;//阈值
        static double ObstacleValueRange = 255;//前方50米,左右各10米

        static int Range = 1;//影响范围

        static int SectorCount = 181;//扇区总数
        public static int Resolution = 1;//扇区角度分辨率
        static int LengthInflationValue = 5;//纵向膨胀系数
        static int WidthInflationValue = 5;//横向膨胀系数
        static int CenterLengthValue = 5 * Length / 6 * 20 + 20;//中心点纵向位置
        static int CenterWidthValue = Width / 2 * 20 + 10;//中心点横向位置

        public byte[,] Matrix = new byte[Length, Width];//存储雷达数据二维数组
        public double[] DirectionObstacleValue = new double[SectorCount];//存储扇区阻值的一维数组
        public byte[] BinaryDirection = new byte[SectorCount];//存储扇区二值的一维数组

        double ObstacleValue = 0;//默认障碍值

        double FirstAngle = 0;//角度，取整
        double SecondAngle = 0;//角度，取整
        #endregion

        #region 方法定义

        /// <summary>
        /// 障碍物膨胀
        /// </summary>
        public void Inflation()
        {
            ///膨胀栅格图的前半部分
            for (int i = 0; i <= 5 * Length / 6; i++)
            {
                for (int j = 0; j < Width; j++)
                {
                    if (Matrix[i, j] == 128)//障碍物节点
                    {
                        if (j == 50)//障碍物在前方
                        {
                            for (int m = i - LengthInflationValue; m <= i + LengthInflationValue; m++)
                            {
                                for (int n = j - WidthInflationValue; n <= j + WidthInflationValue; n++)
                                {
                                    //膨胀的节点在栅格范围内，不是中心点，不是原点，不知障碍物点
                                    if ((m <= 5 * Length / 6 && m >= 0 && n < Width && n >= 0) && !(m == 5 * Length / 6 && n == Width / 2) && !(m == i && n == j) && !(Matrix[m, n] == 128))
                                    {
                                        Matrix[m, n] = 129;
                                    }
                                }
                            }
                        }
                        if (j > 50)//障碍物在左侧
                        {
                            for (int m = i - LengthInflationValue; m <= i + LengthInflationValue; m++)
                            {
                                for (int n = j - WidthInflationValue; n <= j + WidthInflationValue; n++)
                                {
                                    //膨胀的节点在栅格范围内，不是中心点，不是原点，不知障碍物点
                                    if ((m <= 5 * Length / 6 && m >= 0 && n < Width && n >= 0) && !(m == 5 * Length / 6 && n == Width / 2) && !(m == i && n == j) && !(Matrix[m, n] == 128))
                                    {
                                        Matrix[m, n] = 129;
                                    }
                                }
                            }
                        }
                        if (j < 50)//障碍物在右侧
                        {
                            for (int m = i - LengthInflationValue; m <= i + LengthInflationValue; m++)
                            {
                                for (int n = j - WidthInflationValue; n <= j + WidthInflationValue; n++)
                                {
                                    //膨胀的节点在栅格范围内，不是中心点，不是原点，不知障碍物点
                                    if ((m <= 5 * Length / 6 && m >= 0 && n < Width && n >= 0) && !(m == 5 * Length / 6 && n == Width / 2) && !(m == i && n == j) && !(Matrix[m, n] == 128))
                                    {
                                        Matrix[m, n] = 129;
                                    }
                                }
                            }
                        }
                    }
                }
            }           
        }

        /// <summary>
        /// 计算所有扇区障碍值
        /// </summary>
        public void Transformation()
        {
            for (int i = 0; i <= 5 * Length / 6; i++)
            {
                for (int j = 0; j < Width; j++)
                {
                    if (Matrix[i, j] == 128 || Matrix[i, j] == 129)//膨胀后的障碍物点
                    {
                        if (j == Width / 2)//障碍物在前方
                        {
                            //计算障碍物影响的角度范围
                            FirstAngle = ((Math.Atan((1.0 * (CenterLengthValue - (i + 1) * 20)) / (1.0 * ((j + 1) * 20 - CenterWidthValue))) * 180 / 3.14) + 0.5);//右下角
                            SecondAngle = ((Math.Atan((1.0 * (CenterLengthValue - (i + 1) * 20)) / (1.0 * (j * 20 - CenterWidthValue))) * 180 / 3.14) + 180 + 0.5);//左下角
                            //K = (int)(90 + 1.5);
                        }
                        else if (j > Width / 2)//障碍物在右侧
                        {
                            //计算障碍物影响的角度范围
                            FirstAngle = ((Math.Atan((1.0 * (CenterLengthValue - (i + 1) * 20)) / (1.0 * ((j + 1) * 20 - CenterWidthValue))) * 180 / 3.14) + 0.5);//右下角
                            SecondAngle = ((Math.Atan((1.0 * (CenterLengthValue - i * 20)) / (1.0 * (j * 20 - CenterWidthValue))) * 180 / 3.14) + 0.5);//右上角
                            //K = (int)((Math.Atan((1.0 * (L / 2 - i)) / (1.0 * (j - W / 2))) * 180 / 3.14) + 1.5);
                        }
                        else if (j < Width / 2)//障碍物在左侧
                        {
                            //计算障碍物影响的角度范围
                            FirstAngle = ((Math.Atan((1.0 * (CenterLengthValue - i * 20)) / (1.0 * ((j + 1) * 20 - CenterWidthValue))) * 180 / 3.14) + 180 + 0.5);//左上角
                            SecondAngle = ((Math.Atan((1.0 * (CenterLengthValue - (i + 1) * 20)) / (1.0 * (j * 20 - CenterWidthValue))) * 180 / 3.14) + 180 + 0.5);//左下角
                            //K = (int)((Math.Atan((1.0 * (L / 2 - i)) / (1.0 * (j - W / 2))) * 180 / 3.14) + 180 + 1.5);
                        }

                        ObstacleValue = ObstacleValueRange - Math.Sqrt(((5 * Length / 6 - i) * (5 * Length / 6 - i) + (j - Width / 2) * (j - Width / 2)));//计算阻值

                        //更新扇区阻值
                        for (int k = (int)(FirstAngle / Resolution); k <= (int)(SecondAngle / Resolution); k++)
                        {
                            if (ObstacleValue > DirectionObstacleValue[k])
                            {
                                DirectionObstacleValue[k] = ObstacleValue;
                            }
                        }
                    }
                }
            }
        }

        /// <summary>
        /// 扇区障碍值二值化
        /// </summary>
        public void Direction()//判断方向可行性，1为不可行，0为可行。
        {
            for (int i = 0; i < SectorCount; i++)//遍历所有扇区
            {
                Boolean ObstacleExistance = false;
                for (int j = i - Range; j <= i + Range; j++)//判断扇区左右是否可行
                {
                    if (j >= 0 && j < SectorCount)
                    {
                        if (DirectionObstacleValue[j] > ThresholdValue)//大于阈值，不可行扇区
                        {
                            ObstacleExistance = true;
                            j = i + Range + 1;
                        }
                    }
                }

                //二值化处理
                if (ObstacleExistance == true)
                {
                    BinaryDirection[i] = 1;
                }
            }
        }

        /// <summary>
        /// 判断是否可以继续前行
        /// </summary>
        /// <returns></returns>
        public Boolean Access()
        {
            Boolean AccessFlag = false;
            for (int i = 0; i < SectorCount; i++)//判断是否有可行扇区
            {
                if (BinaryDirection[i] == 0)
                {
                    AccessFlag = true;
                }
            }
            return AccessFlag;
        }
        #endregion
    }
}

