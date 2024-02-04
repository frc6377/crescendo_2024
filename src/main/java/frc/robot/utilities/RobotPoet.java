// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import com.github.javaparser.StaticJavaParser;
import com.github.javaparser.ast.*;
import com.github.javaparser.ast.body.*;
import com.github.javaparser.ast.comments.*;
import com.github.javaparser.ast.expr.*;
import com.github.javaparser.ast.modules.*;
import com.github.javaparser.ast.stmt.*;
import com.github.javaparser.ast.visitor.*;
import com.github.javaparser.printer.*;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.UncheckedIOException;
import java.nio.file.Path;
import java.util.ArrayList;

/** Add your docs here. */
public class RobotPoet {
  private static CompilationUnit cu;
  private static String fileSuffix = new String();
  private static ArrayList<String> clazzes = new ArrayList<String>();
  private static ArrayList<String> instances = new ArrayList<String>();

  private RobotPoet() {}

  public static void generateRobotContainerWithout(String... subsystemsToRemove) {
    fileSuffix = "";
    clazzes.clear();
    instances.clear();
    for (String s : subsystemsToRemove) {
      clazzes.add(s);

      // Take the first word of the subsystem name
      // so the output file name doesn't get too long
      fileSuffix += s.charAt(0);
      for (int i = 1; i < s.length(); i++) {
        if (s.charAt(i) >= 'A' && s.charAt(i) <= 'Z') {
          break;
        }
        fileSuffix += s.charAt(i);
      }
    }

    // AutoBuilder requires config setup in the SwerveSubsystem instance.
    // Add for removal since we don't have a better way to detect.
    if (clazzes.contains("SwerveSubsystem")) {
      clazzes.add("AutoBuilder");
    }

    try {
      // Generate the Abstract Syntax Tree (AST) from RobotContainer
      cu = StaticJavaParser.parse(Path.of("src/main/java/frc/robot/RobotContainer.java"));

      // Dependencies to remove are added as they are found.
      // Re-traverse the AST until the list of items to remove stops growing
      int clazzesSize;
      int instancesSize;
      do {
        clazzesSize = clazzes.size();
        instancesSize = instances.size();
        new DeleteVisitor().visit(cu, null);
      } while (clazzesSize != clazzes.size() || instancesSize != instances.size());

      // Create a new file with the result
      try (FileWriter fileWriter =
              new FileWriter("src/main/java/frc/robot/RobotContainerNo" + fileSuffix + ".java");
          PrintWriter printWriter = new PrintWriter(fileWriter)) {
        printWriter.print(cu.toString());
      }

    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  // Creates a new .dot file that can be converted to .svg using Graphiz program
  // The SVG is a web browser viewable image of the AST.
  public static void printAstAsGraph() {
    DotPrinter printer = new DotPrinter(true);
    try (FileWriter fileWriter = new FileWriter("ast.dot");
        PrintWriter printWriter = new PrintWriter(fileWriter)) {
      printWriter.print(printer.output(cu));
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  private static class DeleteVisitor extends ModifierVisitor<String> {

    /* DECLARATORS */
    @Override
    public Visitable visit(VariableDeclarator varDec, String args) {
      VariableDeclarator cpy = varDec.clone();
      Visitable ret = super.visit(varDec, args);
      if (ret == null && !instances.contains(cpy.getNameAsString())) {
        // Found a variable declaration that needs to be removed.
        // Add it to the list
        instances.add(cpy.getNameAsString());
      }
      return ret;
    }

    @Override
    public Visitable visit(final ClassOrInterfaceDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      if (n != null && n.getName().asString().equals("RobotContainer")) {
        // Rename the Class to match the new file getting written
        n.setName("RobotContainerNo" + fileSuffix);
        return n;
      }
      return ret;
    }

    @Override
    public Visitable visit(final ConstructorDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      if (n != null && n.getName().asString().equals("RobotContainer")) {
        // Rename the Constructor to match the new file getting written
        n.setName("RobotContainerNo" + fileSuffix);
        return n;
      }
      return ret;
    }

    @Override
    public Visitable visit(final ReturnStmt n, final String arg) {
      ReturnStmt cpy = n.clone();
      Visitable ret = super.visit(n, arg);
      if (ret == null || n.getExpression().isEmpty()) {
        // The return expression got removed.
        // Can't just remove because then a method would be missing
        // a return value. Return null instead
        return cpy.setExpression(new NullLiteralExpr());
      }
      return ret;
    }

    /* EXPRESSIONS */

    @Override
    public Visitable visit(AssignExpr n, String args) {
      AssignExpr cpy = n.clone();
      Visitable ret = super.visit(n, args);
      if (ret == null && !instances.contains(cpy.getTarget().toString())) {
        // The assignment is being removed, meaning that the
        // thing we were assigning must also get removed. Add it to the list.
        instances.add(cpy.getTarget().toString());
      }
      return ret;
    }

    @Override
    public Visitable visit(final MethodCallExpr n, final String arg) {
      MethodCallExpr cpy = n.clone();
      Visitable ret = super.visit(n, arg);
      if (n == null
          || (n.getScope().isEmpty() && cpy.getScope().isPresent())
          || (n.getArguments().size() != cpy.getArguments().size())) {
        // The scope (class instance or name calling the method) went away
        // or one or more parameters got removed. So this is no longer valid
        // and needs to be removed as well.
        return null;
      }
      return ret;
    }

    @Override
    public Visitable visit(final ObjectCreationExpr n, final String arg) {
      ObjectCreationExpr cpy = n.clone();
      Visitable ret = super.visit(n, arg);
      if (n == null || (n.getArguments().size() != cpy.getArguments().size())) {
        // One or more of the Constructor parameters got removed. Assuming that means
        // this the Constructor is no longer valid and needs to be removed as well.
        return null;
      }
      return ret;
    }

    /* NAMES */
    @Override
    public Visitable visit(final SimpleName n, final String arg) {
      Visitable ret = super.visit(n, arg);
      // Base case for when to propogate an item's removal upward
      // Returning null removes the item from the AST and parent will also be removed (in most
      // cases)
      if (clazzes.contains(n.asString()) || instances.contains(n.asString())) {
        return null;
      }
      return ret;
    }

    /* ERROR CASES */

    // If we find an unparsable statement, should throw an error
    @Override
    public Visitable visit(final UnparsableStmt n, final String arg) throws UncheckedIOException {
      throw new UncheckedIOException("Syntax errors in RobotPoet target", null);
    }

    /* UNSUPPORTED METHODS BELOW */

    // Annotations don't need to be supported, bypass the recursive walk
    @Override
    public Visitable visit(final AnnotationDeclaration n, final String arg) {
      return n;
    }

    @Override
    public Visitable visit(final AnnotationMemberDeclaration n, final String arg) {
      return n;
    }

    // Receiver parameters fall under Annotation use case, which is not supported. Bypass recursive
    // walk
    @Override
    public Visitable visit(final ReceiverParameter n, final String arg) {
      return n;
    }

    // No need to process imports, bypass recursive walk
    @Override
    public Node visit(final ImportDeclaration n, final String arg) {
      return n;
    }

    // No need to process comments, bypass recursive walk
    @Override
    public Visitable visit(final BlockComment n, final String arg) {
      return n;
    }

    @Override
    public Visitable visit(final LineComment n, final String arg) {
      return n;
    }

    @Override
    public Visitable visit(final JavadocComment n, final String arg) {
      return n;
    }

    // No need to process module declarations, bypass recursive walk
    @Override
    public Visitable visit(final ModuleDeclaration n, final String arg) {
      return n;
    }

    @Override
    public Visitable visit(final ModuleRequiresDirective n, final String arg) {
      return n;
    }

    @Override()
    public Visitable visit(final ModuleExportsDirective n, final String arg) {
      return n;
    }

    @Override()
    public Visitable visit(final ModuleProvidesDirective n, final String arg) {
      return n;
    }

    @Override()
    public Visitable visit(final ModuleUsesDirective n, final String arg) {
      return n;
    }

    @Override
    public Visitable visit(final ModuleOpensDirective n, final String arg) {
      return n;
    }
  }
}
