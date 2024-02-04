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
    } catch (IOException e) {
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

    // Annotations don't need to be supported, bypass the recursive walk
    @Override
    public Visitable visit(final AnnotationDeclaration n, final Object arg) {
      return n;
    }

    @Override
    public Visitable visit(final AnnotationMemberDeclaration n, final Object arg) {
      return n;
    }

    @Override
    public Visitable visit(final ArrayAccessExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ArrayCreationExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ArrayInitializerExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final AssertStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final BinaryExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final BlockStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final BooleanLiteralExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final BreakStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final CastExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final CatchClause n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final CharLiteralExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ClassExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ClassOrInterfaceDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final RecordDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ClassOrInterfaceType n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final CompilationUnit n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ConditionalExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ConstructorDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final CompactConstructorDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ContinueStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final DoStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final DoubleLiteralExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final EmptyStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final EnclosedExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    // TODO Uncommon use case
    @Override
    public Visitable visit(final EnumConstantDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    // TODO Uncommon use case
    @Override
    public Visitable visit(final EnumDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ExplicitConstructorInvocationStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final FieldAccessExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final FieldDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ForEachStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ForStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final IfStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final InitializerDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final InstanceOfExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final IntegerLiteralExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final JavadocComment n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final LabeledStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final LongLiteralExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final MarkerAnnotationExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final MemberValuePair n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final MethodCallExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final MethodDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final NameExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final NormalAnnotationExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final NullLiteralExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ObjectCreationExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final PackageDeclaration n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final Parameter n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final Name n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final PrimitiveType n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final SimpleName n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ArrayType n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ArrayCreationLevel n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final IntersectionType n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final UnionType n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ReturnStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final SingleMemberAnnotationExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final StringLiteralExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final SuperExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final SwitchEntry n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final SwitchStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final SynchronizedStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ThisExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final ThrowStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final TryStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final LocalClassDeclarationStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final LocalRecordDeclarationStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }
 
    // Type parameters are part of Class Declaration, should not support. Bypass recursive walk
    @Override
    public Visitable visit(final TypeParameter n, final Object arg) {
      return n;
    }

    @Override
    public Visitable visit(final UnaryExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final UnknownType n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final VariableDeclarationExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final VoidType n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final WhileStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final WildcardType n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final LambdaExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final MethodReferenceExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final TypeExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    /* Covering individual types covers this case as well. Don't Override
    @Override
    public Visitable visit(NodeList n, Object arg) {
      super.visit(n, arg);
      return n;
    }
    */

    // No need to process imports, bypass recursive walk
    @Override
    public Node visit(final ImportDeclaration n, final Object arg) {
      return n;
    }

    // No need to process comments, bypass recursive walk
    @Override
    public Visitable visit(final BlockComment n, final Object arg) {
      return n;
    }

    @Override
    public Visitable visit(final LineComment n, final Object arg) {
      return n;
    }

    // No need to process module declarations, bypass recursive walk
    @Override
    public Visitable visit(final ModuleDeclaration n, final Object arg) {
      return n;
    }

    @Override
    public Visitable visit(final ModuleRequiresDirective n, final Object arg) {
      return n;
    }

    @Override()
    public Visitable visit(final ModuleExportsDirective n, final Object arg) {
      return n;
    }

    @Override()
    public Visitable visit(final ModuleProvidesDirective n, final Object arg) {
      return n;
    }

    @Override()
    public Visitable visit(final ModuleUsesDirective n, final Object arg) {
      return n;
    }

    @Override
    public Visitable visit(final ModuleOpensDirective n, final Object arg) {
      return n;
    }

    // If we find an unparsable statement, should throw an error
    @Override
    public Visitable visit(final UnparsableStmt n, final Object arg)  throws UncheckedIOException {
      throw new UncheckedIOException("Syntax errors in RobotPoet target", null);
    }

    // Receiver parameters fall under Annotation use case, which is not supported. Bypass recursive walk
    @Override
    public Visitable visit(final ReceiverParameter n, final Object arg) {
      return n;
    }

    @Override
    public Visitable visit(final VarType n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final Modifier n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final SwitchExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final YieldStmt n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final TextBlockLiteralExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }

    @Override
    public Visitable visit(final PatternExpr n, final Object arg) {
      super.visit(n, arg);
      return n;
    }
  }
}
