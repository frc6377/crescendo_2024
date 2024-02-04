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
import com.github.javaparser.ast.type.*;
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
    clazzes.clear();
    instances.clear();
    for (String s : subsystemsToRemove) {
      clazzes.add(s);
      fileSuffix += s.charAt(0);
      for (int i = 1; i < s.length(); i++) {
        if (s.charAt(i) >= 'A' && s.charAt(i) <= 'Z') {
          break;
        }
        fileSuffix += s.charAt(i);
      }
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

    @Override
    public Visitable visit(final CompilationUnit n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    /* DECLARATORS */

    @Override
    public Visitable visit(VariableDeclarator varDec, String args) {
      VariableDeclarator cpy = varDec.clone();
      Visitable ret = super.visit(varDec, args);
      if (ret == null && !instances.contains(cpy.getNameAsString())) {
        instances.add(cpy.getNameAsString());
      }
      return ret;
    }

    @Override
    public Visitable visit(final ClassOrInterfaceDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      if (n != null && n.getName().asString().equals("RobotContainer")) {
        n.setName("RobotContainerNo" + fileSuffix);
        return n;
      }
      return ret;
    }

    @Override
    public Visitable visit(final RecordDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ConstructorDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      if (n != null && n.getName().asString().equals("RobotContainer")) {
        n.setName("RobotContainerNo" + fileSuffix);
        return n;
      }
      return ret;
    }

    @Override
    public Visitable visit(final CompactConstructorDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    // TODO Uncommon use case
    @Override
    public Visitable visit(final EnumConstantDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    // TODO Uncommon use case
    @Override
    public Visitable visit(final EnumDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final FieldDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final InitializerDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final MethodDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final PackageDeclaration n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    /* STATEMENTS */

    @Override
    public Visitable visit(ExpressionStmt stmt, String args) {
      Visitable ret = super.visit(stmt, args);
      return ret;
    }

    @Override
    public Visitable visit(final AssertStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final BlockStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final BreakStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ContinueStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final DoStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final EmptyStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ExplicitConstructorInvocationStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ForEachStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ForStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final IfStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final LabeledStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ReturnStmt n, final String arg) {
      ReturnStmt cpy = n.clone();
      Visitable ret = super.visit(n, arg);
      if (ret == null || n.getExpression().isEmpty()) {
        return cpy.setExpression(new NullLiteralExpr());
      }
      return ret;
    }

    @Override
    public Visitable visit(final SwitchStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final SynchronizedStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ThrowStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final TryStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final LocalClassDeclarationStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final LocalRecordDeclarationStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final WhileStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final YieldStmt n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    /* EXPRESSIONS */

    @Override
    public Visitable visit(AssignExpr n, String args) {
      AssignExpr cpy = n.clone();
      Visitable ret = super.visit(n, args);
      if (ret == null && !instances.contains(cpy.getTarget().toString())) {
        instances.add(cpy.getTarget().toString());
      }
      return ret;
    }

    @Override
    public Visitable visit(final ArrayAccessExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ArrayCreationExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ArrayInitializerExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final BinaryExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final BooleanLiteralExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final CastExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final CharLiteralExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ClassExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ConditionalExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final DoubleLiteralExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final EnclosedExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final FieldAccessExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final InstanceOfExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final IntegerLiteralExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final LongLiteralExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final MarkerAnnotationExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final MethodCallExpr n, final String arg) {
      MethodCallExpr cpy = n.clone();
      Visitable ret = super.visit(n, arg);
      if (n == null
          || (n.getScope().isEmpty() && cpy.getScope().isPresent())
          || (n.getArguments().size() != cpy.getArguments().size())) {
        return null;
      }
      return ret;
    }

    @Override
    public Visitable visit(final NameExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final NormalAnnotationExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final NullLiteralExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ObjectCreationExpr n, final String arg) {
      ObjectCreationExpr cpy = n.clone();
      Visitable ret = super.visit(n, arg);
      if (n == null || (n.getArguments().size() != cpy.getArguments().size())) {
        return null;
      }
      return ret;
    }

    @Override
    public Visitable visit(final SingleMemberAnnotationExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final StringLiteralExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final SuperExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ThisExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final UnaryExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final VariableDeclarationExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final LambdaExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final MethodReferenceExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final TypeExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final SwitchExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final TextBlockLiteralExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final PatternExpr n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    /* MISC */
    @Override
    public Visitable visit(final CatchClause n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final MemberValuePair n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ArrayCreationLevel n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final SwitchEntry n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    /* TYPES */

    @Override
    public Visitable visit(final ClassOrInterfaceType n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final PrimitiveType n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final ArrayType n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final IntersectionType n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final UnionType n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final UnknownType n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final VoidType n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final WildcardType n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final VarType n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    /* PARAMETERS */

    @Override
    public Visitable visit(final Parameter n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final TypeParameter n, final String arg) {
      return n;
    }

    /* NAMES */

    @Override
    public Visitable visit(final Name n, final String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }

    @Override
    public Visitable visit(final SimpleName n, final String arg) {
      Visitable ret = super.visit(n, arg);
      if (clazzes.contains(n.asString()) || instances.contains(n.asString())) {
        return null;
      }
      return ret;
    }

    @Override
    public Visitable visit(final Modifier n, final String arg) {
      Visitable ret = super.visit(n, arg);
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

    /* Covering individual types covers this case as well. Don't Override
    @Override
    public Visitable visit(NodeList n, String arg) {
      Visitable ret = super.visit(n, arg);
      return ret;
    }
    */

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
