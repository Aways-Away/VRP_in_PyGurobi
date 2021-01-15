# -*- coding: utf-8 -*-

"""
@author: liu cui
@software: PyCharm
@file: VRP.py
@time: 2021/1/14 14:49
"""

from __future__ import print_function
from __future__ import division, print_function
import numpy as np
from gurobipy import *
import copy


class VRP(object):
    customerNum = 0
    nodeNum = 0
    vehicleNum = 0
    capacity = 0
    cor_X = []
    cor_Y = []
    demand = []
    serviceTime = []
    readyTime = []
    dueTime = []
    disMatrix = [[]]

    def inputData(self):
        data = VRP()
        f = open(file='Data1.txt', mode='r')
        lines = f.readlines()
        count = 0
        for line in lines:
            count = count + 1
            if count == 3:
                line = line.strip()
                str = re.split(r":", line)
                data.vehicleNum = int(str[1])
            elif count == 5:
                line = line.strip()
                str = re.split(r":", line)
                data.customerNum = int(str[1]) - 1  # remove: depot
                data.nodeNum = data.customerNum + 1
            elif count == 7:
                line = line.strip()
                str = re.split(r":", line)
                data.capacity = float(str[1])
            elif 9 <= count < 9 + data.nodeNum:
                line = line.strip()
                str = re.split(r" ", line)
                data.cor_X.append(float(str[1]))
                data.cor_Y.append(float(str[2]))
            elif 60 <= count < 60 + data.nodeNum:
                line = line.strip()
                str = re.split(r" ", line)
                data.demand.append(float(str[1]))
        data.disMatrix = [([0] * data.nodeNum) for p in range(data.nodeNum)]
        for i in range(0, data.nodeNum):
            for j in range(0, data.nodeNum):
                temp = (data.cor_X[i] - data.cor_X[j]) ** 2 + (data.cor_Y[i] - data.cor_Y[j]) ** 2
                data.disMatrix[i][j] = math.sqrt(temp)
                # if i == j:
                #     data.disMatrix[i][j] = 0
                # print("%6.2f" % (math.sqrt(temp)), end=" ")
                temp = 0
        return data

    def printData(self):
        data = self.inputData()
        print("============ print data ============\n")
        print("vehicle number = %4d" % data.vehicleNum)
        print("vehicle capacity = %4d" % data.capacity)
        print("customerNum = %4d" % data.customerNum)
        print("nodeNum = %4d" % data.nodeNum)
        # for i in range(len(data.demand)):
        #     print('{0}\t'.format(data.demand[i]))
        # print("============ distance matrix ============\n")
        # for i in range(data.nodeNum):
        #     for j in range(data.nodeNum):
        #         print("%6.2f" % (data.disMatrix[i][j]), end=" ")
        #     print()

    # def getValue(self):
    #     data = self.inputData()
    #     var_dict = self.solveModel()
    #     nodeNum = data.nodeNum
    #     x_value = np.zeros([nodeNum, nodeNum])
    #     for key in var_dict.keys():
    #         a = key[0]
    #         b = key[1]
    #         x_value[a][b] = var_dict[key].x
    #     return x_value

    # def getRoute(self):
    #     x_value = self.getValue()
    #     x = copy.deepcopy(x_value)
    #     previousPoint = 0
    #     route_temp = [previousPoint]
    #     count = 0
    #     while count != len(x_value) - 1:
    #         print("previousPoint: ", previousPoint)
    #         if x[previousPoint][count] > 0:
    #             previousPoint = count
    #             route_temp.append(previousPoint)
    #             count = 0
    #             continue
    #         else:
    #             count += 1
    #     route_temp.append(len(x_value) - 1)
    #     print("optimal route: ", route_temp)
    #     return route_temp

    def solveModel(self):
        data = self.inputData()
        bigM = 100000
        model = Model('TSP')
        x = {}

        # TODO: create decision variables
        for i in range(data.nodeNum):
            for j in range(data.nodeNum):
                for k in range(data.vehicleNum):
                    if i != j:
                        name = "x_" + str(i) + "_" + str(j) + "_" + str(k)
                        x[i, j, k] = model.addVar(0, 1, vtype=GRB.BINARY, name=name)

        model.update()

        # TODO: set objective function
        obj = LinExpr(0)
        for i in range(data.nodeNum):
            for j in range(data.nodeNum):
                for k in range(data.vehicleNum):
                    if i != j:
                        obj.addTerms(data.disMatrix[i][j], x[i, j, k])
        model.setObjective(obj, GRB.MINIMIZE)

        for k in range(data.vehicleNum):
            lhs = LinExpr(0)
            for j in range(data.nodeNum):
                if j != 0:
                    lhs.addTerms(1, x[0, j, k])
            model.addConstr(lhs == 1, name="vehicle_depart_" + str(k))

        for k in range(data.vehicleNum):
            for h in range(1, data.nodeNum - 1):
                expr1 = LinExpr(0)
                expr2 = LinExpr(0)
                for i in range(data.nodeNum):
                    if h != i:
                        expr1.addTerms(1, x[i, h, k])
                for j in range(data.nodeNum):
                    if h != j:
                        expr2.addTerms(1, x[h, j, k])

                model.addConstr(expr1 == expr2, name="flow_conservation_" + str(i))
                expr1.clear()
                expr2.clear()

        for k in range(data.vehicleNum):
            lhs = LinExpr(0)
            for j in range(data.nodeNum - 1):
                if j != 0:
                    lhs.addTerms(1, x[j, data.nodeNum - 1, k])
            model.addConstr(lhs == 1, name="vehicle_depart_" + str(k))

        for i in range(1, data.nodeNum - 1):
            lhs = LinExpr(0)
            for k in range(data.vehicleNum):
                for j in range(1, data.nodeNum):
                    if i != j:
                        lhs.addTerms(1, x[i, j, k])
            model.addConstr(lhs == 1, name="customer_visit_" + str(i))

        for k in range(data.vehicleNum):
            lhs = LinExpr(0)
            for i in range(1, data.nodeNum - 1):
                for j in range(data.nodeNum):
                    if i != j:
                        lhs.addTerms(data.demand[i], x[i, j, k])
            model.addConstr(lhs <= data.capacity, name="capacity_vehicle" + str(k))

        # TODO: start slove model
        model.write("VRP.lp")
        model.setParam(GRB.Param.MIPGap, 0)
        model.setParam(GRB.Param.TimeLimit, 60)
        model.optimize()

        print("============ optimal value ============\n")

        # TODO: return best route
        if model.status == GRB.OPTIMAL:
            print("Best Solution: ", model.objVal, "\n")
            var = model.getVars()

        print(model.Objval)
        for key in x.keys():
            if x[key].x > 0:
                print(x[key].VarName + " = ", x[key].x)

        return x

    def start(self):
        self.inputData()
        self.printData()
        self.solveModel()
        # self.getValue()
        # self.getRoute()


if __name__ == '__main__':
    vrp = VRP()
    vrp.start()
