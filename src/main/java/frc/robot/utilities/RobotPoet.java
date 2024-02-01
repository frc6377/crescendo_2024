// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.github.javaparser.StaticJavaParser;
import com.github.javaparser.ast.CompilationUnit;
import com.github.javaparser.ast.Node;
import com.github.javaparser.ast.body.VariableDeclarator;
import com.github.javaparser.ast.expr.AssignExpr;
import com.github.javaparser.ast.stmt.ExpressionStmt;
import com.github.javaparser.ast.visitor.ModifierVisitor;
import com.github.javaparser.printer.DotPrinter;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.util.ArrayList;

/** Add your docs here. */
public class RobotPoet {
  private CompilationUnit cu;

  private static ArrayList<String> clazzes = new ArrayList<String>();
  private static ArrayList<String> instances = new ArrayList<String>();

  public RobotPoet(String... subsystemsToRemove) {
    for (String s : subsystemsToRemove) {
      clazzes.add(s);
    }
    try {
      cu = StaticJavaParser.parse(Path.of("src/main/java/frc/robot/RobotContainer.java"));
      int clazzesSize;
      int instancesSize;
      do {
        clazzesSize = clazzes.size();
        instancesSize = instances.size();
        new DeleteVisitor().visit(cu, null);
      } while (clazzesSize != clazzes.size() || instancesSize != instances.size());

      try (FileWriter fileWriter = new FileWriter("src/main/java/frc/robot/RobotContainer.java");
          PrintWriter printWriter = new PrintWriter(fileWriter)) {
        printWriter.print(cu.toString());
      }

    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  public void printAstAsGraph() {
    DotPrinter printer = new DotPrinter(true);
    try (FileWriter fileWriter = new FileWriter("ast.dot");
        PrintWriter printWriter = new PrintWriter(fileWriter)) {
        printWriter.print(printer.output(cu));
    }
     catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  private class DeleteVisitor extends ModifierVisitor<Object> {

    @Override
    public Node visit(ExpressionStmt stmt, Object args) {
      super.visit(stmt, args);
      if (stmt.getExpression() == null) {
        return null;
      }
      for (String sub : clazzes) {
        if (stmt.getExpression().toString().contains(sub)) {
          return null;
        }
      }
      for (String inst : instances) {
        if (stmt.getExpression().toString().contains(inst)) {
          return null;
        }
      }
      return stmt;
    }

    @Override
    public Node visit(VariableDeclarator varDec, Object args) {
      super.visit(varDec, args);
      if (varDec.getNameAsExpression() == null) {
        return null;
      }
      for (String sub : clazzes) {
        String t = varDec.getTypeAsString();
        if (t.equals(sub)) {
          instances.add(varDec.getNameAsString());
          return null;
        }
      }
      for (String inst : instances) {
        if (varDec.getNameAsString().equals(inst)) {
          if (!clazzes.contains(varDec.getTypeAsString())) {
            clazzes.add(varDec.getTypeAsString());
          }
          return null;
        }
      }
      return varDec;
    }

    @Override
    public Node visit(AssignExpr expr, Object args) {
      super.visit(expr, args);
      for (String inst : instances) {
        if (expr.getValue().toString().contains(inst)) {
          instances.add(expr.getTarget().toString());
          return null;
        }
      }

      for (String sub : clazzes) {
        if (expr.getValue().toString().contains(sub)) {
          instances.add(expr.getTarget().toString());
        }
      }
      return expr;
    }
  }
}
